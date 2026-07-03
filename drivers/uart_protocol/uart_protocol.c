/**
 * @file uart_protocol.c
 * @brief HXZP 串口帧协议 driver 实现。
 */
#define DT_DRV_COMPAT hxzp_uart_protocol

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/uart_protocol.h>

LOG_MODULE_REGISTER(hxzp_uart_protocol, LOG_LEVEL_INF);

/** @brief 串口接收中断单次读取长度，单位字节。 */
#define UART_PROTOCOL_IRQ_READ_SIZE 32U

/** @brief 响应状态字段长度，单位字节。 */
#define UART_PROTOCOL_STATUS_SIZE 2U

/** @brief 帧头后固定头部长度，单位字节。 */
#define UART_PROTOCOL_HEADER_SIZE 8U

/** @brief CRC16 初始值。 */
#define UART_PROTOCOL_CRC16_INIT 0xFFFFU

/** @brief CRC16 多项式。 */
#define UART_PROTOCOL_CRC16_POLY 0x1021U

/**
 * @brief 帧解析状态。
 */
enum uart_protocol_parse_state
{
    /**< 等待帧头第 1 字节。 */
    UART_PROTOCOL_PARSE_WAIT_SOF1 = 0,
    /**< 等待帧头第 2 字节。 */
    UART_PROTOCOL_PARSE_WAIT_SOF2,
    /**< 读取固定头部。 */
    UART_PROTOCOL_PARSE_HEADER,
    /**< 读取负载数据。 */
    UART_PROTOCOL_PARSE_PAYLOAD,
    /**< 读取 CRC16。 */
    UART_PROTOCOL_PARSE_CRC,
};

/**
 * @brief 帧解析器运行数据。
 */
struct uart_protocol_parser
{
    /**< 当前解析状态。 */
    enum uart_protocol_parse_state state;
    /**< 固定头部缓冲区。 */
    uint8_t header[UART_PROTOCOL_HEADER_SIZE];
    /**< 固定头部已接收长度，单位字节。 */
    uint16_t header_pos;
    /**< 负载已接收长度，单位字节。 */
    uint16_t payload_pos;
    /**< CRC16 已接收长度，单位字节。 */
    uint16_t crc_pos;
    /**< 接收到的 CRC16。 */
    uint16_t rx_crc;
    /**< 当前协议帧。 */
    struct uart_protocol_frame frame;
};

/**
 * @brief 串口协议设备配置。
 */
struct uart_protocol_config
{
    /**< 绑定的底层 UART 设备。 */
    const struct device *uart_dev;
    /**< 是否回显原始接收字节。 */
    bool echo_rx_raw;
};

/**
 * @brief 串口协议设备运行数据。
 */
struct uart_protocol_data
{
    /**< 接收环形缓冲区。 */
    struct ring_buf rx_ring;
    /**< 接收环形缓冲区存储。 */
    uint8_t *rx_ring_buffer;
    /**< 接收环形缓冲区长度。 */
    size_t rx_ring_size;
    /**< 命令表。 */
    const struct uart_protocol_cmd_entry *entries;
    /**< 命令表项数量。 */
    size_t entry_count;
    /**< 帧解析器运行数据。 */
    struct uart_protocol_parser parser;
    /**< 协议状态互斥锁。 */
    struct k_mutex mutex;
    /**< 发送互斥锁。 */
    struct k_mutex tx_mutex;
    /**< 接收信号量。 */
    struct k_sem rx_sem;
    /**< 协议线程控制块。 */
    struct k_thread thread_data;
    /**< 协议线程栈。 */
    k_thread_stack_t *thread_stack;
    /**< 协议线程栈大小。 */
    size_t thread_stack_size;
};

/**
 * @brief 读取小端 16 位整数。
 * @param data 数据指针。
 * @return uint16_t 读取结果。
 */
static uint16_t uart_protocol_get_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief 写入小端 16 位整数。
 * @param data 数据指针。
 * @param value 待写入的值。
 * @return void
 */
static void uart_protocol_put_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/**
 * @brief 更新 CRC16。
 * @param crc 当前 CRC16。
 * @param data 输入字节。
 * @return uint16_t 更新后的 CRC16。
 */
static uint16_t uart_protocol_crc16_update(uint16_t crc, uint8_t data)
{
    uint8_t i;

    crc ^= (uint16_t)data << 8;

    for (i = 0U; i < 8U; i++)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (crc << 1) ^ UART_PROTOCOL_CRC16_POLY;
        }
        else
        {
            crc <<= 1;
        }
    }

    return crc;
}

/**
 * @brief 计算 CRC16。
 * @param data 数据指针。
 * @param len 数据长度，单位字节。
 * @return uint16_t CRC16。
 */
static uint16_t uart_protocol_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = UART_PROTOCOL_CRC16_INIT;
    size_t i;

    for (i = 0U; i < len; i++)
    {
        crc = uart_protocol_crc16_update(crc, data[i]);
    }

    return crc;
}

/**
 * @brief 重置帧解析器。
 * @param parser 帧解析器。
 * @return void
 */
static void uart_protocol_parser_reset(struct uart_protocol_parser *parser)
{
    memset(parser, 0, sizeof(*parser));
    parser->state = UART_PROTOCOL_PARSE_WAIT_SOF1;
}

/**
 * @brief 判断串口协议设备是否可用。
 * @param dev 串口协议设备。
 * @return int 0 表示可用，负值表示不可用。
 */
static int uart_protocol_check_device(const struct device *dev)
{
    if (dev == NULL)
    {
        return -EINVAL;
    }

    if (!device_is_ready(dev))
    {
        return -ENODEV;
    }

    return 0;
}

/**
 * @brief 查找命令表项。
 * @param data 串口协议设备运行数据。
 * @param cmd 命令号。
 * @return const struct uart_protocol_cmd_entry * 找到的命令表项，未找到返回 NULL。
 */
static const struct uart_protocol_cmd_entry *uart_protocol_find_entry(
    const struct uart_protocol_data *data,
    uint16_t cmd)
{
    size_t i;

    for (i = 0U; i < data->entry_count; i++)
    {
        if (data->entries[i].cmd == cmd)
        {
            return &data->entries[i];
        }
    }

    return NULL;
}

/**
 * @brief 分发已完成校验的协议帧。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @return void
 */
static void uart_protocol_dispatch_frame(const struct device *dev,
                                         const struct uart_protocol_frame *frame)
{
    struct uart_protocol_data *data = dev->data;
    const struct uart_protocol_cmd_entry *entry;
    uart_protocol_handler_t handler;
    void *user_data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    entry = uart_protocol_find_entry(data, frame->cmd);

    if ((entry == NULL) || (entry->handler == NULL))
    {
        k_mutex_unlock(&data->mutex);
        LOG_WRN("UART command not found: cmd=0x%04x seq=%u", frame->cmd, frame->seq);
        uart_protocol_send_response(dev, frame->cmd, frame->seq, -ENOENT, NULL, 0U);
        return;
    }

    handler = entry->handler;
    user_data = entry->user_data;
    k_mutex_unlock(&data->mutex);

    if (handler(dev, frame, user_data) < 0)
    {
        LOG_WRN("UART command handler failed: cmd=0x%04x seq=%u", frame->cmd, frame->seq);
    }
}

/**
 * @brief 校验并提交协议帧。
 * @param dev 串口协议设备。
 * @param parser 帧解析器。
 * @return void
 */
static void uart_protocol_submit_frame(const struct device *dev,
                                       struct uart_protocol_parser *parser)
{
    uint8_t crc_buffer[UART_PROTOCOL_HEADER_SIZE + UART_PROTOCOL_MAX_PAYLOAD_LEN];
    uint16_t calc_crc;

    memcpy(crc_buffer, parser->header, UART_PROTOCOL_HEADER_SIZE);

    if (parser->frame.payload_len > 0U)
    {
        memcpy(&crc_buffer[UART_PROTOCOL_HEADER_SIZE],
               parser->frame.payload,
               parser->frame.payload_len);
    }

    calc_crc = uart_protocol_crc16(crc_buffer,
                                   UART_PROTOCOL_HEADER_SIZE + parser->frame.payload_len);

    if (calc_crc != parser->rx_crc)
    {
        LOG_WRN("UART frame crc mismatch: cmd=0x%04x seq=%u calc=0x%04x rx=0x%04x",
                parser->frame.cmd,
                parser->frame.seq,
                calc_crc,
                parser->rx_crc);
        return;
    }

    uart_protocol_dispatch_frame(dev, &parser->frame);
}

/**
 * @brief 完成固定头部解析。
 * @param parser 帧解析器。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_protocol_finish_header(struct uart_protocol_parser *parser)
{
    parser->frame.version = parser->header[0];
    parser->frame.flags = parser->header[1];
    parser->frame.cmd = uart_protocol_get_le16(&parser->header[2]);
    parser->frame.seq = uart_protocol_get_le16(&parser->header[4]);
    parser->frame.payload_len = uart_protocol_get_le16(&parser->header[6]);

    if (parser->frame.version != UART_PROTOCOL_VERSION)
    {
        LOG_WRN("UART frame version mismatch: version=%u", parser->frame.version);
        return -EPROTO;
    }

    if (parser->frame.payload_len > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        LOG_WRN("UART frame payload too large: len=%u", parser->frame.payload_len);
        return -EMSGSIZE;
    }

    return 0;
}

/**
 * @brief 向帧解析器输入 1 字节。
 * @param dev 串口协议设备。
 * @param parser 帧解析器。
 * @param byte 输入字节。
 * @return void
 */
static void uart_protocol_parse_byte(const struct device *dev,
                                     struct uart_protocol_parser *parser,
                                     uint8_t byte)
{
    switch (parser->state)
    {
    case UART_PROTOCOL_PARSE_WAIT_SOF1:
        if (byte == UART_PROTOCOL_SOF1)
        {
            parser->state = UART_PROTOCOL_PARSE_WAIT_SOF2;
        }
        break;

    case UART_PROTOCOL_PARSE_WAIT_SOF2:
        if (byte == UART_PROTOCOL_SOF2)
        {
            parser->state = UART_PROTOCOL_PARSE_HEADER;
            parser->header_pos = 0U;
        }
        else if (byte != UART_PROTOCOL_SOF1)
        {
            parser->state = UART_PROTOCOL_PARSE_WAIT_SOF1;
        }
        break;

    case UART_PROTOCOL_PARSE_HEADER:
        parser->header[parser->header_pos] = byte;
        parser->header_pos++;

        if (parser->header_pos >= UART_PROTOCOL_HEADER_SIZE)
        {
            if (uart_protocol_finish_header(parser) < 0)
            {
                uart_protocol_parser_reset(parser);
            }
            else if (parser->frame.payload_len == 0U)
            {
                parser->state = UART_PROTOCOL_PARSE_CRC;
                parser->crc_pos = 0U;
                parser->rx_crc = 0U;
            }
            else
            {
                parser->state = UART_PROTOCOL_PARSE_PAYLOAD;
                parser->payload_pos = 0U;
            }
        }
        break;

    case UART_PROTOCOL_PARSE_PAYLOAD:
        parser->frame.payload[parser->payload_pos] = byte;
        parser->payload_pos++;

        if (parser->payload_pos >= parser->frame.payload_len)
        {
            parser->state = UART_PROTOCOL_PARSE_CRC;
            parser->crc_pos = 0U;
            parser->rx_crc = 0U;
        }
        break;

    case UART_PROTOCOL_PARSE_CRC:
        if (parser->crc_pos == 0U)
        {
            parser->rx_crc = byte;
            parser->crc_pos = 1U;
        }
        else
        {
            parser->rx_crc |= (uint16_t)byte << 8;
            uart_protocol_submit_frame(dev, parser);
            uart_protocol_parser_reset(parser);
        }
        break;

    default:
        uart_protocol_parser_reset(parser);
        break;
    }
}

/**
 * @brief 串口中断回调。
 * @param uart_dev 底层串口设备。
 * @param user_data 用户参数，指向串口协议设备。
 * @return void
 */
static void uart_protocol_irq_callback(const struct device *uart_dev, void *user_data)
{
    const struct device *dev = user_data;
    struct uart_protocol_data *data = dev->data;
    uint8_t rx_data[UART_PROTOCOL_IRQ_READ_SIZE];

    while (uart_irq_update(uart_dev) && uart_irq_rx_ready(uart_dev))
    {
        int rx_len;
        uint32_t put_len;

        rx_len = uart_fifo_read(uart_dev, rx_data, sizeof(rx_data));

        if (rx_len <= 0)
        {
            continue;
        }

        LOG_HEXDUMP_DBG(rx_data, (size_t)rx_len, "UART rx raw");

        put_len = ring_buf_put(&data->rx_ring, rx_data, (uint32_t)rx_len);

        if (put_len < (uint32_t)rx_len)
        {
            LOG_WRN("UART rx ring overflow");
        }

        k_sem_give(&data->rx_sem);
    }
}

/**
 * @brief 串口协议线程入口。
 * @param arg1 串口协议设备。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void uart_protocol_thread_entry(void *arg1, void *arg2, void *arg3)
{
    const struct device *dev = arg1;
    const struct uart_protocol_config *config = dev->config;
    struct uart_protocol_data *data = dev->data;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (1)
    {
        uint8_t byte;

        k_sem_take(&data->rx_sem, K_FOREVER);

        while (ring_buf_get(&data->rx_ring, &byte, 1U) == 1U)
        {
            if (config->echo_rx_raw)
            {
                uart_poll_out(config->uart_dev, byte);
            }

            uart_protocol_parse_byte(dev, &data->parser, byte);
        }
    }
}

/**
 * @brief 初始化串口协议设备。
 * @param dev 串口协议设备。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_protocol_init(const struct device *dev)
{
    const struct uart_protocol_config *config = dev->config;
    struct uart_protocol_data *data = dev->data;

    if (!device_is_ready(config->uart_dev))
    {
        return -ENODEV;
    }

    k_mutex_init(&data->mutex);
    k_mutex_init(&data->tx_mutex);
    k_sem_init(&data->rx_sem, 0, 1);
    ring_buf_init(&data->rx_ring, data->rx_ring_size, data->rx_ring_buffer);
    uart_protocol_parser_reset(&data->parser);

    k_thread_create(&data->thread_data,
                    data->thread_stack,
                    data->thread_stack_size,
                    uart_protocol_thread_entry,
                    (void *)dev,
                    NULL,
                    NULL,
                    CONFIG_HXZP_UART_PROTOCOL_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    uart_irq_callback_user_data_set(config->uart_dev,
                                    uart_protocol_irq_callback,
                                    (void *)dev);
    uart_irq_rx_enable(config->uart_dev);

    LOG_INF("UART protocol init done");
    return 0;
}

/**
 * @brief 注册串口协议命令表。
 * @param dev 串口协议设备。
 * @param entries 命令表。
 * @param entry_count 命令表项数量。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_register_handlers(const struct device *dev,
                                    const struct uart_protocol_cmd_entry *entries,
                                    size_t entry_count)
{
    struct uart_protocol_data *data;
    int ret;

    ret = uart_protocol_check_device(dev);

    if (ret < 0)
    {
        return ret;
    }

    if ((entries == NULL) || (entry_count == 0U))
    {
        return -EINVAL;
    }

    data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->entries = entries;
    data->entry_count = entry_count;
    k_mutex_unlock(&data->mutex);

    return 0;
}

/**
 * @brief 清空串口协议命令表。
 * @param dev 串口协议设备。
 * @return void
 */
void uart_protocol_clear_handlers(const struct device *dev)
{
    struct uart_protocol_data *data;

    if (uart_protocol_check_device(dev) < 0)
    {
        return;
    }

    data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->entries = NULL;
    data->entry_count = 0U;
    k_mutex_unlock(&data->mutex);
}

/**
 * @brief 发送串口原始调试数据。
 * @param dev 串口协议设备。
 * @param tx_data 待发送数据指针。
 * @param len 待发送数据长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_send_raw(const struct device *dev, const uint8_t *tx_data, uint16_t len)
{
    const struct uart_protocol_config *config;
    struct uart_protocol_data *data;
    uint16_t i;
    int ret;

    ret = uart_protocol_check_device(dev);

    if (ret < 0)
    {
        return ret;
    }

    if ((tx_data == NULL) && (len > 0U))
    {
        return -EINVAL;
    }

    config = dev->config;
    data = dev->data;

    k_mutex_lock(&data->tx_mutex, K_FOREVER);

    for (i = 0U; i < len; i++)
    {
        uart_poll_out(config->uart_dev, tx_data[i]);
    }

    k_mutex_unlock(&data->tx_mutex);

    return 0;
}

/**
 * @brief 发送串口协议帧。
 * @param dev 串口协议设备。
 * @param cmd 命令号。
 * @param seq 帧序号。
 * @param flags 协议标志位。
 * @param payload 负载数据。
 * @param payload_len 负载长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_send_frame(const struct device *dev,
                             uint16_t cmd,
                             uint16_t seq,
                             uint8_t flags,
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    const struct uart_protocol_config *config;
    struct uart_protocol_data *data;
    uint8_t frame_buffer[2U + UART_PROTOCOL_HEADER_SIZE + UART_PROTOCOL_MAX_PAYLOAD_LEN + 2U];
    uint16_t crc;
    size_t pos = 0U;
    size_t crc_start;
    size_t i;
    int ret;

    ret = uart_protocol_check_device(dev);

    if (ret < 0)
    {
        return ret;
    }

    if ((payload == NULL) && (payload_len > 0U))
    {
        return -EINVAL;
    }

    if (payload_len > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        return -EMSGSIZE;
    }

    config = dev->config;
    data = dev->data;

    frame_buffer[pos] = UART_PROTOCOL_SOF1;
    pos++;
    frame_buffer[pos] = UART_PROTOCOL_SOF2;
    pos++;

    crc_start = pos;

    frame_buffer[pos] = UART_PROTOCOL_VERSION;
    pos++;
    frame_buffer[pos] = flags;
    pos++;
    uart_protocol_put_le16(&frame_buffer[pos], cmd);
    pos += 2U;
    uart_protocol_put_le16(&frame_buffer[pos], seq);
    pos += 2U;
    uart_protocol_put_le16(&frame_buffer[pos], payload_len);
    pos += 2U;

    if (payload_len > 0U)
    {
        memcpy(&frame_buffer[pos], payload, payload_len);
        pos += payload_len;
    }

    crc = uart_protocol_crc16(&frame_buffer[crc_start], pos - crc_start);
    uart_protocol_put_le16(&frame_buffer[pos], crc);
    pos += 2U;

    k_mutex_lock(&data->tx_mutex, K_FOREVER);

    for (i = 0U; i < pos; i++)
    {
        uart_poll_out(config->uart_dev, frame_buffer[i]);
    }

    k_mutex_unlock(&data->tx_mutex);

    return 0;
}

/**
 * @brief 发送命令响应帧。
 * @param dev 串口协议设备。
 * @param req_cmd 请求命令号。
 * @param seq 请求帧序号。
 * @param status 运行状态。
 * @param payload 响应负载数据。
 * @param payload_len 响应负载长度，单位字节。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_protocol_send_response(const struct device *dev,
                                uint16_t req_cmd,
                                uint16_t seq,
                                int16_t status,
                                const uint8_t *payload,
                                uint16_t payload_len)
{
    uint8_t rsp_payload[UART_PROTOCOL_MAX_PAYLOAD_LEN];
    uint16_t rsp_payload_len;

    if ((payload == NULL) && (payload_len > 0U))
    {
        return -EINVAL;
    }

    if ((payload_len + UART_PROTOCOL_STATUS_SIZE) > UART_PROTOCOL_MAX_PAYLOAD_LEN)
    {
        return -EMSGSIZE;
    }

    uart_protocol_put_le16(rsp_payload, (uint16_t)status);
    rsp_payload_len = UART_PROTOCOL_STATUS_SIZE;

    if (payload_len > 0U)
    {
        memcpy(&rsp_payload[rsp_payload_len], payload, payload_len);
        rsp_payload_len += payload_len;
    }

    return uart_protocol_send_frame(dev,
                                    req_cmd | UART_PROTOCOL_RSP_FLAG,
                                    seq,
                                    0U,
                                    rsp_payload,
                                    rsp_payload_len);
}

#define UART_PROTOCOL_DEFINE(inst)                                                     \
    static uint8_t uart_protocol_rx_ring_buffer_##inst[DT_INST_PROP(inst, rx_ring_size)]; \
    static K_KERNEL_STACK_DEFINE(uart_protocol_thread_stack_##inst,                     \
                                 CONFIG_HXZP_UART_PROTOCOL_THREAD_STACK_SIZE);          \
    static struct uart_protocol_data uart_protocol_data_##inst = {                      \
        .rx_ring_buffer = uart_protocol_rx_ring_buffer_##inst,                          \
        .rx_ring_size = sizeof(uart_protocol_rx_ring_buffer_##inst),                    \
        .thread_stack = uart_protocol_thread_stack_##inst,                              \
        .thread_stack_size = K_KERNEL_STACK_SIZEOF(uart_protocol_thread_stack_##inst),  \
    };                                                                                  \
    static const struct uart_protocol_config uart_protocol_config_##inst = {            \
        .uart_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart)),                         \
        .echo_rx_raw = DT_INST_PROP(inst, echo_rx_raw),                                 \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(inst,                                                         \
                          uart_protocol_init,                                           \
                          NULL,                                                         \
                          &uart_protocol_data_##inst,                                   \
                          &uart_protocol_config_##inst,                                 \
                          POST_KERNEL,                                                  \
                          CONFIG_HXZP_UART_PROTOCOL_INIT_PRIORITY,                      \
                          NULL)

DT_INST_FOREACH_STATUS_OKAY(UART_PROTOCOL_DEFINE)
