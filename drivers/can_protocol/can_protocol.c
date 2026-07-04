/**
 * @file can_protocol.c
 * @brief HXZP CAN 协议传输层 driver 实现。
 */
#define DT_DRV_COMPAT hxzp_can_protocol

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(hxzp_can_protocol, LOG_LEVEL_INF);

/** @brief CAN 发送入队等待时间。 */
#define CAN_PROTOCOL_TX_QUEUE_TIMEOUT K_MSEC(100)

/** @brief CAN 发送超时时间。 */
#define CAN_PROTOCOL_TX_TIMEOUT K_MSEC(100)

/** @brief 无效过滤器 ID。 */
#define CAN_PROTOCOL_INVALID_FILTER_ID (-1)

/**
 * @brief CAN 协议传输层接收事件。
 */
struct can_protocol_rx_event
{
    /**< 接收到的 CAN 报文。 */
    struct can_frame frame;
    /**< 过滤器槽位索引。 */
    uint8_t slot;
};

/**
 * @brief CAN 协议传输层接收槽位。
 */
struct can_protocol_rx_slot
{
    /**< 槽位是否正在使用。 */
    bool used;
    /**< 硬件过滤器 ID。 */
    int hw_filter_id;
    /**< 过滤器配置。 */
    struct can_filter filter;
    /**< 接收回调函数。 */
    can_protocol_rx_handler_t handler;
    /**< 接收回调用户参数。 */
    void *user_data;
};

/**
 * @brief CAN 协议传输层硬件过滤器回调上下文。
 */
struct can_protocol_rx_context
{
    /**< CAN 协议传输层设备。 */
    const struct device *dev;
    /**< 过滤器槽位索引。 */
    uint8_t slot;
};

/**
 * @brief CAN 协议传输层设备配置。
 */
struct can_protocol_config
{
    /**< 底层 CAN 控制器设备。 */
    const struct device *can_dev;
    /**< CAN 波特率。 */
    uint32_t bitrate;
};

/**
 * @brief CAN 协议传输层运行数据。
 */
struct can_protocol_data
{
    /**< 设备自身指针。 */
    const struct device *self;
    /**< 接收消息队列。 */
    struct k_msgq rx_msgq;
    /**< 发送消息队列。 */
    struct k_msgq tx_msgq;
    /**< 接收消息队列缓冲区。 */
    char *rx_msgq_buffer;
    /**< 发送消息队列缓冲区。 */
    char *tx_msgq_buffer;
    /**< 接收队列长度。 */
    size_t rx_queue_len;
    /**< 发送队列长度。 */
    size_t tx_queue_len;
    /**< 接收槽位表。 */
    struct can_protocol_rx_slot *slots;
    /**< 接收回调上下文表。 */
    struct can_protocol_rx_context *contexts;
    /**< 接收槽位数量。 */
    size_t slot_count;
    /**< 互斥锁。 */
    struct k_mutex lock;
    /**< 接收线程控制块。 */
    struct k_thread rx_thread;
    /**< 发送线程控制块。 */
    struct k_thread tx_thread;
    /**< 接收线程栈。 */
    k_thread_stack_t *rx_stack;
    /**< 发送线程栈。 */
    k_thread_stack_t *tx_stack;
    /**< 接收线程栈大小。 */
    size_t rx_stack_size;
    /**< 发送线程栈大小。 */
    size_t tx_stack_size;
    /**< 设备是否已经初始化完成。 */
    bool initialized;
};

/**
 * @brief CAN 控制器状态变化回调。
 * @param can_dev CAN 设备。
 * @param state 新的总线状态。
 * @param err_cnt 当前总线错误计数。
 * @param user_data 用户参数。
 * @return void
 */
static void can_protocol_state_change_cb(const struct device *can_dev,
                                         enum can_state state,
                                         struct can_bus_err_cnt err_cnt,
                                         void *user_data)
{
    ARG_UNUSED(can_dev);
    ARG_UNUSED(user_data);

    LOG_WRN("CAN state changed: state=%d tx_err=%u rx_err=%u",
            state,
            err_cnt.tx_err_cnt,
            err_cnt.rx_err_cnt);
}

/**
 * @brief CAN 接收硬件过滤器回调。
 * @param can_dev CAN 设备。
 * @param frame 接收到的 CAN 报文。
 * @param user_data 用户参数。
 * @return void
 */
static void can_protocol_rx_callback(const struct device *can_dev,
                                     struct can_frame *frame,
                                     void *user_data)
{
    const struct can_protocol_rx_context *context = user_data;
    const struct device *dev;
    uint8_t slot;
    struct can_protocol_data *data;
    struct can_protocol_rx_event event;
    int ret;

    ARG_UNUSED(can_dev);

    if ((context == NULL) || (frame == NULL))
    {
        return;
    }

    dev = context->dev;
    slot = context->slot;

    if (dev == NULL)
    {
        return;
    }

    data = dev->data;

    if (slot >= data->slot_count)
    {
        return;
    }

    memset(&event, 0, sizeof(event));
    memcpy(&event.frame, frame, sizeof(event.frame));
    event.slot = slot;

    ret = k_msgq_put(&data->rx_msgq, &event, K_NO_WAIT);

    if (ret < 0)
    {
        LOG_WRN("CAN rx queue full: id=0x%03x slot=%u", frame->id, slot);
    }
}

/**
 * @brief CAN 接收分发线程入口。
 * @param arg1 设备指针。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void can_protocol_rx_thread_entry(void *arg1, void *arg2, void *arg3)
{
    const struct device *dev = arg1;
    struct can_protocol_data *data = dev->data;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        struct can_protocol_rx_event event;
        can_protocol_rx_handler_t handler;
        void *user_data;

        if (k_msgq_get(&data->rx_msgq, &event, K_FOREVER) != 0)
        {
            continue;
        }

        k_mutex_lock(&data->lock, K_FOREVER);

        if (!data->initialized ||
            (event.slot >= data->slot_count) ||
            !data->slots[event.slot].used ||
            (data->slots[event.slot].handler == NULL))
        {
            k_mutex_unlock(&data->lock);
            continue;
        }

        handler = data->slots[event.slot].handler;
        user_data = data->slots[event.slot].user_data;
        k_mutex_unlock(&data->lock);

        handler(&event.frame, user_data);
    }
}

/**
 * @brief CAN 发送线程入口。
 * @param arg1 设备指针。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void can_protocol_tx_thread_entry(void *arg1, void *arg2, void *arg3)
{
    const struct device *dev = arg1;
    const struct can_protocol_config *config = dev->config;
    struct can_protocol_data *data = dev->data;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        struct can_frame frame;
        bool initialized;
        int ret;

        if (k_msgq_get(&data->tx_msgq, &frame, K_FOREVER) != 0)
        {
            continue;
        }

        k_mutex_lock(&data->lock, K_FOREVER);
        initialized = data->initialized;
        k_mutex_unlock(&data->lock);

        if (!initialized)
        {
            continue;
        }

        ret = can_send(config->can_dev, &frame, CAN_PROTOCOL_TX_TIMEOUT, NULL, NULL);

        if (ret < 0)
        {
            LOG_ERR("CAN send failed: id=0x%03x ret=%d", frame.id, ret);
        }
    }
}

/**
 * @brief 找到空闲过滤器槽位。
 * @param data CAN 协议传输层运行数据。
 * @return int 成功返回槽位索引，失败返回负错误码。
 */
static int can_protocol_find_free_slot(struct can_protocol_data *data)
{
    size_t i;

    for (i = 0U; i < data->slot_count; i++)
    {
        if (!data->slots[i].used)
        {
            return (int)i;
        }
    }

    return -ENOMEM;
}

/**
 * @brief 初始化 CAN 协议传输层设备。
 * @param dev CAN 协议传输层设备。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_protocol_init(const struct device *dev)
{
    const struct can_protocol_config *config = dev->config;
    struct can_protocol_data *data = dev->data;
    int ret;

    if (!device_is_ready(config->can_dev))
    {
        return -ENODEV;
    }

    data->self = dev;
    k_mutex_init(&data->lock);
    k_msgq_init(&data->rx_msgq,
                data->rx_msgq_buffer,
                sizeof(struct can_protocol_rx_event),
                data->rx_queue_len);
    k_msgq_init(&data->tx_msgq,
                data->tx_msgq_buffer,
                sizeof(struct can_frame),
                data->tx_queue_len);

    ret = can_stop(config->can_dev);

    if ((ret < 0) && (ret != -EALREADY))
    {
        return ret;
    }

    /*
     * CAN_MODE_NORMAL 不包含 CAN_MODE_ONE_SHOT，因此控制器保持自动重传。
     */
    ret = can_set_mode(config->can_dev, CAN_MODE_NORMAL);

    if (ret < 0)
    {
        return ret;
    }

    ret = can_set_bitrate(config->can_dev, config->bitrate);

    if (ret < 0)
    {
        return ret;
    }

    can_set_state_change_callback(config->can_dev,
                                  can_protocol_state_change_cb,
                                  NULL);

    ret = can_start(config->can_dev);

    if (ret < 0)
    {
        can_set_state_change_callback(config->can_dev, NULL, NULL);
        return ret;
    }

    k_thread_create(&data->rx_thread,
                    data->rx_stack,
                    data->rx_stack_size,
                    can_protocol_rx_thread_entry,
                    (void *)dev,
                    NULL,
                    NULL,
                    CONFIG_HXZP_CAN_PROTOCOL_RX_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&data->tx_thread,
                    data->tx_stack,
                    data->tx_stack_size,
                    can_protocol_tx_thread_entry,
                    (void *)dev,
                    NULL,
                    NULL,
                    CONFIG_HXZP_CAN_PROTOCOL_TX_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    k_mutex_lock(&data->lock, K_FOREVER);
    data->initialized = true;
    k_mutex_unlock(&data->lock);

    LOG_DBG("CAN protocol ready: bitrate=%u tx_queue=%u rx_queue=%u filters=%u",
            config->bitrate,
            (unsigned int)data->tx_queue_len,
            (unsigned int)data->rx_queue_len,
            (unsigned int)data->slot_count);

    return 0;
}

/**
 * @brief 向 CAN 协议传输层注册接收过滤器。
 * @param dev CAN 协议传输层设备。
 * @param filter 过滤器配置。
 * @param rx_handler 接收回调函数。
 * @param user_data 接收回调用户参数。
 * @return int 成功时返回过滤器句柄，失败时返回负错误码。
 */
int can_protocol_add_rx_filter(const struct device *dev,
                               const struct can_protocol_filter_config *filter,
                               can_protocol_rx_handler_t rx_handler,
                               void *user_data)
{
    const struct can_protocol_config *config;
    struct can_protocol_data *data;
    struct can_filter zephyr_filter;
    int slot;
    int hw_filter_id;

    if ((dev == NULL) || (filter == NULL) || (rx_handler == NULL))
    {
        return -EINVAL;
    }

    if (!device_is_ready(dev))
    {
        return -ENODEV;
    }

    config = dev->config;
    data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    slot = can_protocol_find_free_slot(data);

    if (slot < 0)
    {
        k_mutex_unlock(&data->lock);
        return slot;
    }

    zephyr_filter.id = filter->id;
    zephyr_filter.mask = filter->mask;
    zephyr_filter.flags = filter->flags;
    data->contexts[slot].dev = dev;
    data->contexts[slot].slot = (uint8_t)slot;
    hw_filter_id = can_add_rx_filter(config->can_dev,
                                     can_protocol_rx_callback,
                                     &data->contexts[slot],
                                     &zephyr_filter);

    if (hw_filter_id < 0)
    {
        k_mutex_unlock(&data->lock);
        return hw_filter_id;
    }

    data->slots[slot].used = true;
    data->slots[slot].hw_filter_id = hw_filter_id;
    data->slots[slot].filter = zephyr_filter;
    data->slots[slot].handler = rx_handler;
    data->slots[slot].user_data = user_data;
    k_mutex_unlock(&data->lock);

    return slot;
}

/**
 * @brief 从 CAN 协议传输层移除接收过滤器。
 * @param dev CAN 协议传输层设备。
 * @param filter_handle 过滤器句柄。
 * @return void
 */
void can_protocol_remove_rx_filter(const struct device *dev, int filter_handle)
{
    const struct can_protocol_config *config;
    struct can_protocol_data *data;
    int hw_filter_id;

    if (dev == NULL)
    {
        return;
    }

    config = dev->config;
    data = dev->data;

    if ((filter_handle < 0) || ((size_t)filter_handle >= data->slot_count))
    {
        return;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    if (!data->slots[filter_handle].used)
    {
        k_mutex_unlock(&data->lock);
        return;
    }

    hw_filter_id = data->slots[filter_handle].hw_filter_id;
    memset(&data->slots[filter_handle], 0, sizeof(data->slots[filter_handle]));
    data->slots[filter_handle].hw_filter_id = CAN_PROTOCOL_INVALID_FILTER_ID;
    memset(&data->contexts[filter_handle], 0, sizeof(data->contexts[filter_handle]));
    k_mutex_unlock(&data->lock);

    can_remove_rx_filter(config->can_dev, hw_filter_id);
}

/**
 * @brief 将 CAN 报文放入发送队列。
 * @param dev CAN 协议传输层设备。
 * @param frame 待发送的 CAN 报文。
 * @return int 0 表示成功放入队列，负值表示失败。
 */
int can_protocol_send(const struct device *dev, const struct can_frame *frame)
{
    struct can_protocol_data *data;
    bool initialized;
    int ret;

    if ((dev == NULL) || (frame == NULL))
    {
        return -EINVAL;
    }

    if (frame->dlc > CAN_MAX_DLC)
    {
        return -EMSGSIZE;
    }

    data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);
    initialized = data->initialized;
    k_mutex_unlock(&data->lock);

    if (!initialized)
    {
        return -ENODEV;
    }

    ret = k_msgq_put(&data->tx_msgq,
                     frame,
                     CAN_PROTOCOL_TX_QUEUE_TIMEOUT);

    if (ret < 0)
    {
        LOG_ERR("CAN tx queue put failed: id=0x%03x ret=%d", frame->id, ret);
    }

    return ret;
}

/**
 * @brief 将标准帧数据报文放入发送队列。
 * @param dev CAN 协议传输层设备。
 * @param id 标准帧 ID。
 * @param data 待发送的数据。
 * @param dlc 数据长度。
 * @return int 0 表示成功放入队列，负值表示失败。
 */
int can_protocol_send_data(const struct device *dev,
                           uint32_t id,
                           const uint8_t *data,
                           uint8_t dlc)
{
    struct can_frame frame;

    if ((data == NULL) && (dlc > 0U))
    {
        return -EINVAL;
    }

    if (dlc > CAN_MAX_DLC)
    {
        return -EMSGSIZE;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.dlc = dlc;
    frame.flags = 0U;

    if (dlc > 0U)
    {
        memcpy(frame.data, data, dlc);
    }

    return can_protocol_send(dev, &frame);
}

#define CAN_PROTOCOL_DEFINE(inst)                                                       \
    static char can_protocol_rx_msgq_buffer_##inst[                                     \
        DT_INST_PROP(inst, rx_queue_len) * sizeof(struct can_protocol_rx_event)];       \
    static char can_protocol_tx_msgq_buffer_##inst[                                     \
        DT_INST_PROP(inst, tx_queue_len) * sizeof(struct can_frame)];                   \
    static struct can_protocol_rx_slot can_protocol_slots_##inst[                       \
        CONFIG_HXZP_CAN_PROTOCOL_MAX_RX_FILTERS];                                      \
    static struct can_protocol_rx_context can_protocol_contexts_##inst[                 \
        CONFIG_HXZP_CAN_PROTOCOL_MAX_RX_FILTERS];                                      \
    static K_KERNEL_STACK_DEFINE(can_protocol_rx_stack_##inst,                          \
                                 CONFIG_HXZP_CAN_PROTOCOL_RX_THREAD_STACK_SIZE);        \
    static K_KERNEL_STACK_DEFINE(can_protocol_tx_stack_##inst,                          \
                                 CONFIG_HXZP_CAN_PROTOCOL_TX_THREAD_STACK_SIZE);        \
    static struct can_protocol_data can_protocol_data_##inst = {                        \
        .rx_msgq_buffer = can_protocol_rx_msgq_buffer_##inst,                           \
        .tx_msgq_buffer = can_protocol_tx_msgq_buffer_##inst,                           \
        .rx_queue_len = DT_INST_PROP(inst, rx_queue_len),                               \
        .tx_queue_len = DT_INST_PROP(inst, tx_queue_len),                               \
        .slots = can_protocol_slots_##inst,                                             \
        .contexts = can_protocol_contexts_##inst,                                       \
        .slot_count = ARRAY_SIZE(can_protocol_slots_##inst),                            \
        .rx_stack = can_protocol_rx_stack_##inst,                                       \
        .tx_stack = can_protocol_tx_stack_##inst,                                       \
        .rx_stack_size = K_KERNEL_STACK_SIZEOF(can_protocol_rx_stack_##inst),           \
        .tx_stack_size = K_KERNEL_STACK_SIZEOF(can_protocol_tx_stack_##inst),           \
    };                                                                                  \
    static const struct can_protocol_config can_protocol_config_##inst = {              \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can)),                           \
        .bitrate = DT_INST_PROP(inst, bitrate),                                         \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(inst,                                                         \
                          can_protocol_init,                                            \
                          NULL,                                                         \
                          &can_protocol_data_##inst,                                    \
                          &can_protocol_config_##inst,                                  \
                          POST_KERNEL,                                                  \
                          CONFIG_HXZP_CAN_PROTOCOL_INIT_PRIORITY,                       \
                          NULL)

DT_INST_FOREACH_STATUS_OKAY(CAN_PROTOCOL_DEFINE)
