/**
 * @file gen_motor.c
 * @brief HXZP gen-motor driver 实现。
 */
#define DT_DRV_COMPAT hxzp_gen_motor

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can_protocol.h>
#include <zephyr/drivers/gen_motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(hxzp_gen_motor, LOG_LEVEL_INF);

/** @brief 管理节点 ID。 */
#define GEN_MOTOR_MANAGEMENT_NODE_ID 0x7EU

/** @brief 协议允许的最大节点 ID。 */
#define GEN_MOTOR_MAX_NODE_ID 0x7FU

/** @brief 协议允许的最小业务节点 ID。 */
#define GEN_MOTOR_MIN_BUSINESS_NODE_ID 0x01U

/** @brief 主机命令帧 ID 基值。 */
#define GEN_MOTOR_HOST_CMD_BASE 0x100U

/** @brief 管理命令帧 ID。 */
#define GEN_MOTOR_MANAGEMENT_CMD_ID 0x17EU

/** @brief 管理应答帧 ID。 */
#define GEN_MOTOR_MANAGEMENT_ACK_ID 0x1FEU

/** @brief 管理上报帧 ID。 */
#define GEN_MOTOR_MANAGEMENT_REPORT_ID 0x27EU

/** @brief 应答帧 ID 基值。 */
#define GEN_MOTOR_ACK_BASE 0x180U

/** @brief 上报帧 ID 基值。 */
#define GEN_MOTOR_REPORT_BASE 0x200U

/** @brief 电机协议过滤器组掩码。 */
#define GEN_MOTOR_ID_GROUP_MASK 0x780U

/** @brief 主动上报周期最小值，单位毫秒。 */
#define GEN_MOTOR_REPORT_PERIOD_MIN_MS 2U

/** @brief 主动上报周期最大值，单位毫秒。 */
#define GEN_MOTOR_REPORT_PERIOD_MAX_MS 60000U

/** @brief 复位 CAN 配置确认码。 */
#define GEN_MOTOR_RESET_CAN_CONFIG_CONFIRM_CODE 0xA5U

/** @brief gen-motor 过滤器数量。 */
#define GEN_MOTOR_FILTER_COUNT 2U

/**
 * @brief 电机协议命令码定义。
 */
enum gen_motor_protocol_command
{
    /**< 设置运行使能和控制模式。 */
    GEN_MOTOR_CMD_SET_RUN_MODE = 0x01,
    /**< 设置电流目标值。 */
    GEN_MOTOR_CMD_SET_CURRENT_TARGET = 0x02,
    /**< 设置速度目标值。 */
    GEN_MOTOR_CMD_SET_SPEED_TARGET = 0x03,
    /**< 设置位置目标值。 */
    GEN_MOTOR_CMD_SET_POSITION_TARGET = 0x04,
    /**< 写速度环 PID 参数。 */
    GEN_MOTOR_CMD_SET_SPEED_PID = 0x10,
    /**< 读速度环 PID 参数。 */
    GEN_MOTOR_CMD_READ_SPEED_PID = 0x11,
    /**< 写位置环 PID 参数。 */
    GEN_MOTOR_CMD_SET_POSITION_PID = 0x12,
    /**< 读位置环 PID 参数。 */
    GEN_MOTOR_CMD_READ_POSITION_PID = 0x13,
    /**< 设置节点 ID。 */
    GEN_MOTOR_CMD_SET_NODE_ID = 0x20,
    /**< 读取当前节点 ID。 */
    GEN_MOTOR_CMD_READ_NODE_ID = 0x21,
    /**< 读取 App 版本号。 */
    GEN_MOTOR_CMD_READ_APP_VERSION = 0x22,
    /**< 设置主动上报配置。 */
    GEN_MOTOR_CMD_SET_REPORT_CONFIG = 0x23,
    /**< 读取主动上报配置。 */
    GEN_MOTOR_CMD_READ_REPORT_CONFIG = 0x24,
    /**< 复位 CAN 配置信息。 */
    GEN_MOTOR_CMD_RESET_CAN_CONFIG = 0x25,
    /**< 设置 FOC 基础配置。 */
    GEN_MOTOR_CMD_SET_FOC_CONFIG = 0x26,
    /**< 读取 FOC 基础配置。 */
    GEN_MOTOR_CMD_READ_FOC_CONFIG = 0x27,
    /**< 触发零点校准。 */
    GEN_MOTOR_CMD_ZERO_CALIBRATION = 0x30,
    /**< 读取零点信息。 */
    GEN_MOTOR_CMD_READ_ZERO = 0x31,
    /**< 未配置发现上报。 */
    GEN_MOTOR_CMD_MANAGEMENT_DISCOVERY = 0x60,
    /**< 未配置电机识别灯效。 */
    GEN_MOTOR_CMD_MANAGEMENT_IDENTIFY = 0x61,
    /**< 配置未配置电机业务节点 ID。 */
    GEN_MOTOR_CMD_MANAGEMENT_CONFIGURE_ID = 0x62,
};

/**
 * @brief 电机协议应答状态码。
 */
enum gen_motor_protocol_status_code
{
    /**< 命令执行成功。 */
    GEN_MOTOR_STATUS_OK = 0x00,
};

/**
 * @brief 接收帧分类。
 */
enum gen_motor_frame_kind
{
    /**< 未知帧。 */
    GEN_MOTOR_FRAME_UNKNOWN = 0,
    /**< 管理应答帧。 */
    GEN_MOTOR_FRAME_MANAGEMENT_ACK,
    /**< 管理发现上报帧。 */
    GEN_MOTOR_FRAME_MANAGEMENT_REPORT,
    /**< 应答帧。 */
    GEN_MOTOR_FRAME_ACK,
    /**< 主动上报帧。 */
    GEN_MOTOR_FRAME_REPORT,
};

/**
 * @brief gen-motor 设备配置。
 */
struct gen_motor_config
{
    /**< CAN 协议传输层设备。 */
    const struct device *can_protocol_dev;
};

/**
 * @brief gen-motor 设备运行数据。
 */
struct gen_motor_data
{
    /**< 设备自身指针。 */
    const struct device *self;
    /**< 电机对象表。 */
    struct gen_motor *motors;
    /**< 电机对象数量。 */
    size_t motor_count;
    /**< 最近一次未配置发现上报缓存。 */
    struct gen_motor_discovery last_discovery;
    /**< 互斥锁。 */
    struct k_mutex lock;
    /**< 接收过滤器句柄。 */
    int filter_handles[GEN_MOTOR_FILTER_COUNT];
    /**< 待接收的管理应答命令码。 */
    uint8_t pending_management_ack_cmd;
    /**< 是否存在待接收的管理应答。 */
    bool pending_management_ack_valid;
    /**< 是否已经初始化完成。 */
    bool initialized;
};

/**
 * @brief 电机对象定义。
 */
struct gen_motor
{
    /**< 所属 gen-motor 设备。 */
    const struct device *dev;
    /**< 电机对象名称。 */
    const char *name;
    /**< 电机在对象表中的索引。 */
    size_t index;
    /**< 电机默认节点 ID。 */
    uint8_t default_node_id;
    /**< 电机默认主动上报周期，单位毫秒。 */
    uint16_t default_report_period_ms;
    /**< 状态缓存。 */
    struct gen_motor_state state;
    /**< 是否已经打印过状态上报日志。 */
    bool state_report_logged;
    /**< 普通命令待应答位图。 */
    uint64_t pending_ack_mask;
    /**< 进入 OTA 前的上报使能。 */
    bool ota_report_enabled_before_ota;
    /**< 进入 OTA 前的上报周期。 */
    uint16_t ota_report_period_before_ota;
    /**< 是否存在可恢复的 OTA 上报配置。 */
    bool ota_report_restore_valid;
};

/**
 * @brief 判断节点 ID 是否有效。
 * @param node_id 节点 ID。
 * @return bool true 表示有效，false 表示无效。
 */
static bool gen_motor_node_id_is_valid(uint8_t node_id)
{
    if (node_id < GEN_MOTOR_MIN_BUSINESS_NODE_ID)
    {
        return false;
    }

    if (node_id > GEN_MOTOR_MAX_NODE_ID)
    {
        return false;
    }

    if (node_id == GEN_MOTOR_MANAGEMENT_NODE_ID)
    {
        return false;
    }

    return true;
}

/**
 * @brief 生成主机命令帧 ID。
 * @param node_id 目标节点 ID。
 * @return uint32_t 主机命令帧 ID。
 */
static uint32_t gen_motor_host_cmd_id(uint8_t node_id)
{
    return GEN_MOTOR_HOST_CMD_BASE + node_id;
}

/**
 * @brief 写入 32 位短 UID。
 * @param payload 协议负载缓冲区。
 * @param offset 写入偏移。
 * @param uid32 短 UID。
 * @return void
 */
static void gen_motor_put_uid32(uint8_t payload[CAN_MAX_DLC], size_t offset, uint32_t uid32)
{
    sys_put_le32(uid32, &payload[offset]);
}

/**
 * @brief 判断 FOC 基础配置参数编号是否有效。
 * @param param FOC 基础配置参数编号。
 * @return bool true 表示有效，false 表示无效。
 */
static bool gen_motor_foc_config_param_is_valid(enum gen_motor_foc_config_param param)
{
    return param <= GEN_MOTOR_FOC_CONFIG_PHASE_MAP;
}

/**
 * @brief 判断主动上报周期是否有效。
 * @param period_ms 主动上报周期，单位毫秒。
 * @return bool true 表示有效，false 表示无效。
 */
static bool gen_motor_report_period_is_valid(uint16_t period_ms)
{
    if (period_ms < GEN_MOTOR_REPORT_PERIOD_MIN_MS)
    {
        return false;
    }

    if (period_ms > GEN_MOTOR_REPORT_PERIOD_MAX_MS)
    {
        return false;
    }

    return true;
}

/**
 * @brief 判断普通命令是否可以记录待应答状态。
 * @param cmd 命令码。
 * @return bool true 表示可以记录，false 表示不能记录。
 */
static bool gen_motor_ack_cmd_can_be_pending(uint8_t cmd)
{
    return cmd < 64U;
}

/**
 * @brief 生成普通命令待应答位。
 * @param cmd 命令码。
 * @return uint64_t 待应答位掩码。
 */
static uint64_t gen_motor_ack_pending_bit(uint8_t cmd)
{
    return UINT64_C(1) << cmd;
}

/**
 * @brief 记录普通命令待应答状态。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @return void
 */
static void gen_motor_mark_pending_ack_locked(struct gen_motor *motor, uint8_t cmd)
{
    if (!gen_motor_ack_cmd_can_be_pending(cmd))
    {
        return;
    }

    motor->pending_ack_mask |= gen_motor_ack_pending_bit(cmd);
}

/**
 * @brief 取走匹配的普通命令待应答状态。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @return bool true 表示存在匹配待应答，false 表示不存在。
 */
static bool gen_motor_take_pending_ack_locked(struct gen_motor *motor, uint8_t cmd)
{
    uint64_t mask;

    if (!gen_motor_ack_cmd_can_be_pending(cmd))
    {
        return false;
    }

    mask = gen_motor_ack_pending_bit(cmd);

    if ((motor->pending_ack_mask & mask) == 0U)
    {
        return false;
    }

    motor->pending_ack_mask &= ~mask;

    return true;
}

/**
 * @brief 记录管理命令待应答状态。
 * @param data gen-motor 运行数据。
 * @param cmd 命令码。
 * @return void
 */
static void gen_motor_mark_pending_management_ack_locked(struct gen_motor_data *data,
                                                         uint8_t cmd)
{
    data->pending_management_ack_cmd = cmd;
    data->pending_management_ack_valid = true;
}

/**
 * @brief 取走匹配的管理命令待应答状态。
 * @param data gen-motor 运行数据。
 * @param cmd 命令码。
 * @return bool true 表示存在匹配待应答，false 表示不存在。
 */
static bool gen_motor_take_pending_management_ack_locked(struct gen_motor_data *data,
                                                         uint8_t cmd)
{
    if (!data->pending_management_ack_valid)
    {
        return false;
    }

    if (data->pending_management_ack_cmd != cmd)
    {
        return false;
    }

    data->pending_management_ack_valid = false;

    return true;
}

/**
 * @brief 判断电机对象是否有效。
 * @param motor 电机对象。
 * @return bool true 表示有效，false 表示无效。
 */
static bool gen_motor_is_valid(const struct gen_motor *motor)
{
    const struct device *dev;
    const struct gen_motor_data *data;

    if ((motor == NULL) || (motor->dev == NULL))
    {
        return false;
    }

    dev = motor->dev;
    data = dev->data;

    if (motor->index >= data->motor_count)
    {
        return false;
    }

    return (&data->motors[motor->index] == motor);
}

/**
 * @brief 初始化单个电机对象状态。
 * @param motor 电机对象。
 * @return void
 */
static void gen_motor_reset_one_locked(struct gen_motor *motor)
{
    const char *name = motor->name;
    const struct device *dev = motor->dev;
    size_t index = motor->index;
    uint8_t default_node_id = motor->default_node_id;
    uint16_t default_report_period_ms = motor->default_report_period_ms;

    memset(motor, 0, sizeof(*motor));
    motor->dev = dev;
    motor->name = name;
    motor->index = index;
    motor->default_node_id = default_node_id;
    motor->default_report_period_ms = default_report_period_ms;
    motor->state.node_id = default_node_id;
    motor->state.configured = true;
    motor->state.requested_node_id = default_node_id;
    motor->state.requested_mode = GEN_MOTOR_CONTROL_MODE_SPEED;

    if (gen_motor_report_period_is_valid(default_report_period_ms))
    {
        motor->state.report_period_ms = default_report_period_ms;
    }
    else
    {
        motor->state.report_period_ms = GEN_MOTOR_REPORT_PERIOD_MIN_MS;
    }
}

/**
 * @brief 重置全部电机状态。
 * @param data gen-motor 运行数据。
 * @return void
 */
static void gen_motor_reset_all_locked(struct gen_motor_data *data)
{
    size_t i;

    for (i = 0U; i < data->motor_count; i++)
    {
        gen_motor_reset_one_locked(&data->motors[i]);
    }

    memset(&data->last_discovery, 0, sizeof(data->last_discovery));
    data->pending_management_ack_cmd = 0U;
    data->pending_management_ack_valid = false;
}

/**
 * @brief 按当前节点或待切换节点 ID 查找电机对象。
 * @param data gen-motor 运行数据。
 * @param node_id 节点 ID。
 * @return struct gen_motor * 找到时返回电机对象，未找到返回 NULL。
 */
static struct gen_motor *gen_motor_find_by_any_node_locked(struct gen_motor_data *data,
                                                           uint8_t node_id)
{
    size_t i;

    for (i = 0U; i < data->motor_count; i++)
    {
        if (data->motors[i].state.node_id == node_id)
        {
            return &data->motors[i];
        }

        if (data->motors[i].state.pending_node_id_change &&
            (data->motors[i].state.requested_node_id == node_id))
        {
            return &data->motors[i];
        }
    }

    return NULL;
}

/**
 * @brief 判断节点 ID 是否已被占用。
 * @param data gen-motor 运行数据。
 * @param node_id 待检查的节点 ID。
 * @param ignore_index 检查时忽略的电机索引。
 * @return bool true 表示已占用，false 表示可用。
 */
static bool gen_motor_node_id_is_reserved_locked(struct gen_motor_data *data,
                                                 uint8_t node_id,
                                                 size_t ignore_index)
{
    size_t i;

    for (i = 0U; i < data->motor_count; i++)
    {
        if (i == ignore_index)
        {
            continue;
        }

        if (data->motors[i].state.node_id == node_id)
        {
            return true;
        }

        if (data->motors[i].state.pending_node_id_change &&
            (data->motors[i].state.requested_node_id == node_id))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 解码接收帧 ID。
 * @param frame_id 接收帧 ID。
 * @param kind 输出帧分类。
 * @param node_id 输出节点 ID。
 * @return bool true 表示成功，false 表示不属于当前电机协议。
 */
static bool gen_motor_decode_frame_id(uint32_t frame_id,
                                      enum gen_motor_frame_kind *kind,
                                      uint8_t *node_id)
{
    if ((kind == NULL) || (node_id == NULL))
    {
        return false;
    }

    if (frame_id == GEN_MOTOR_MANAGEMENT_ACK_ID)
    {
        *kind = GEN_MOTOR_FRAME_MANAGEMENT_ACK;
        *node_id = GEN_MOTOR_MANAGEMENT_NODE_ID;
        return true;
    }

    if (frame_id == GEN_MOTOR_MANAGEMENT_REPORT_ID)
    {
        *kind = GEN_MOTOR_FRAME_MANAGEMENT_REPORT;
        *node_id = GEN_MOTOR_MANAGEMENT_NODE_ID;
        return true;
    }

    if ((frame_id >= GEN_MOTOR_ACK_BASE) &&
        (frame_id <= (GEN_MOTOR_ACK_BASE + GEN_MOTOR_MAX_NODE_ID)))
    {
        *kind = GEN_MOTOR_FRAME_ACK;
        *node_id = (uint8_t)(frame_id - GEN_MOTOR_ACK_BASE);
        return true;
    }

    if ((frame_id >= GEN_MOTOR_REPORT_BASE) &&
        (frame_id <= (GEN_MOTOR_REPORT_BASE + GEN_MOTOR_MAX_NODE_ID)))
    {
        *kind = GEN_MOTOR_FRAME_REPORT;
        *node_id = (uint8_t)(frame_id - GEN_MOTOR_REPORT_BASE);
        return true;
    }

    *kind = GEN_MOTOR_FRAME_UNKNOWN;
    *node_id = 0U;
    return false;
}

/**
 * @brief 记录发送结果。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @param ret 发送结果。
 * @return void
 */
static void gen_motor_record_send_result_locked(struct gen_motor *motor, uint8_t cmd, int ret)
{
    if (ret < 0)
    {
        motor->state.last_can_error = ret;
        return;
    }

    motor->state.last_can_error = 0;
    motor->state.tx_count++;
    gen_motor_mark_pending_ack_locked(motor, cmd);
}

/**
 * @brief 记录管理命令发送结果。
 * @param data gen-motor 运行数据。
 * @param cmd 命令码。
 * @param ret 发送结果。
 * @return void
 */
static void gen_motor_record_management_send_result_locked(struct gen_motor_data *data,
                                                           uint8_t cmd,
                                                           int ret)
{
    if (data->motor_count > 0U)
    {
        if (ret < 0)
        {
            data->motors[0].state.last_can_error = ret;
        }
        else
        {
            data->motors[0].state.last_can_error = 0;
            data->motors[0].state.tx_count++;
        }
    }

    if (ret < 0)
    {
        return;
    }

    gen_motor_mark_pending_management_ack_locked(data, cmd);
}

/**
 * @brief 按指定节点 ID 发送协议负载。
 * @param motor 电机对象。
 * @param node_id 发送时使用的节点 ID。
 * @param payload 待发送的 8 字节协议负载。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_payload(struct gen_motor *motor,
                                  uint8_t node_id,
                                  const uint8_t payload[CAN_MAX_DLC])
{
    const struct gen_motor_config *config = motor->dev->config;
    struct gen_motor_data *data = motor->dev->data;
    int ret;

    ret = can_protocol_send_data(config->can_protocol_dev,
                                 gen_motor_host_cmd_id(node_id),
                                 payload,
                                 CAN_MAX_DLC);

    k_mutex_lock(&data->lock, K_FOREVER);
    gen_motor_record_send_result_locked(motor, payload[0], ret);
    k_mutex_unlock(&data->lock);

    return ret;
}

/**
 * @brief 发送管理协议负载。
 * @param dev gen-motor 设备。
 * @param payload 待发送的 8 字节管理协议负载。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_management_payload(const struct device *dev,
                                             const uint8_t payload[CAN_MAX_DLC])
{
    const struct gen_motor_config *config = dev->config;
    struct gen_motor_data *data = dev->data;
    int ret;

    ret = can_protocol_send_data(config->can_protocol_dev,
                                 GEN_MOTOR_MANAGEMENT_CMD_ID,
                                 payload,
                                 CAN_MAX_DLC);

    k_mutex_lock(&data->lock, K_FOREVER);

    gen_motor_record_management_send_result_locked(data, payload[0], ret);

    k_mutex_unlock(&data->lock);

    return ret;
}

/**
 * @brief 获取发送所需的可写电机对象和节点 ID。
 * @param motor 电机对象。
 * @param writable_motor 输出可写电机对象。
 * @param node_id 输出当前节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_prepare_send(const struct gen_motor *motor,
                                  struct gen_motor **writable_motor,
                                  uint8_t *node_id)
{
    struct gen_motor_data *data;

    if ((writable_motor == NULL) || (node_id == NULL))
    {
        return -EINVAL;
    }

    if (!gen_motor_is_valid(motor))
    {
        return -EINVAL;
    }

    data = motor->dev->data;
    k_mutex_lock(&data->lock, K_FOREVER);
    *writable_motor = &data->motors[motor->index];
    *node_id = (*writable_motor)->state.node_id;
    k_mutex_unlock(&data->lock);

    return 0;
}

/**
 * @brief 发送无参数命令。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_simple(const struct gen_motor *motor, uint8_t cmd)
{
    struct gen_motor *writable_motor;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    return gen_motor_send_payload(writable_motor, node_id, payload);
}

/**
 * @brief 发送带参数编号和 int32 值的命令。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @param param 参数编号。
 * @param value 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_param_write(const struct gen_motor *motor,
                                      uint8_t cmd,
                                      enum gen_motor_pid_param param,
                                      int32_t value)
{
    struct gen_motor *writable_motor;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    payload[1] = (uint8_t)param;
    sys_put_le32((uint32_t)value, &payload[2]);

    return gen_motor_send_payload(writable_motor, node_id, payload);
}

/**
 * @brief 发送带参数编号和 uint32 值的命令。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @param param 参数编号。
 * @param value 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_u32_param_write(const struct gen_motor *motor,
                                          uint8_t cmd,
                                          uint8_t param,
                                          uint32_t value)
{
    struct gen_motor *writable_motor;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    payload[1] = param;
    sys_put_le32(value, &payload[2]);

    return gen_motor_send_payload(writable_motor, node_id, payload);
}

/**
 * @brief 发送带参数编号的读取命令。
 * @param motor 电机对象。
 * @param cmd 命令码。
 * @param param 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_send_u8_param_read(const struct gen_motor *motor,
                                        uint8_t cmd,
                                        uint8_t param)
{
    struct gen_motor *writable_motor;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    payload[1] = param;

    return gen_motor_send_payload(writable_motor, node_id, payload);
}

/**
 * @brief 处理接收到的应答帧。
 * @param motor 电机对象。
 * @param frame 应答帧。
 * @return void
 */
static void gen_motor_handle_ack_locked(struct gen_motor *motor,
                                        const struct can_frame *frame)
{
    struct gen_motor_state *state = &motor->state;
    size_t payload_size;

    if (!gen_motor_take_pending_ack_locked(motor, frame->data[0]))
    {
        return;
    }

    state->last_ack.cmd = frame->data[0];
    state->last_ack.status = frame->data[1];
    state->ack_valid = true;
    state->ack_count++;

    if (state->last_ack.status != GEN_MOTOR_STATUS_OK)
    {
        LOG_WRN("%s ack failed: cmd=0x%02x status=0x%02x",
                motor->name,
                state->last_ack.cmd,
                state->last_ack.status);
    }

    memset(state->last_ack.payload, 0, sizeof(state->last_ack.payload));

    if (frame->dlc > 2U)
    {
        payload_size = MIN((size_t)frame->dlc - 2U,
                           sizeof(state->last_ack.payload));
        memcpy(state->last_ack.payload, &frame->data[2], payload_size);
    }

    if ((frame->data[0] == GEN_MOTOR_CMD_SET_NODE_ID) &&
        state->pending_node_id_change)
    {
        if (frame->data[1] == GEN_MOTOR_STATUS_OK)
        {
            state->node_id = state->requested_node_id;
            state->configured = true;
        }
        else
        {
            state->requested_node_id = state->node_id;
        }

        state->pending_node_id_change = false;
    }

    if ((frame->data[0] == GEN_MOTOR_CMD_READ_NODE_ID) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK))
    {
        state->reported_node_id = state->last_ack.payload[0];
        state->reported_node_id_valid = true;
    }

    if ((frame->data[0] == GEN_MOTOR_CMD_READ_APP_VERSION) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK) &&
        (frame->dlc >= 5U))
    {
        state->app_version.major = frame->data[2];
        state->app_version.minor = frame->data[3];
        state->app_version.patch = frame->data[4];
        state->app_version_valid = true;

        LOG_INF("motor version: name=%s version=%u.%u.%u",
                motor->name,
                state->app_version.major,
                state->app_version.minor,
                state->app_version.patch);
    }

    if (((frame->data[0] == GEN_MOTOR_CMD_SET_REPORT_CONFIG) ||
         (frame->data[0] == GEN_MOTOR_CMD_READ_REPORT_CONFIG)) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK) &&
        (frame->dlc >= 5U))
    {
        state->report_enabled = (frame->data[2] != 0U);
        state->report_period_ms = sys_get_le16(&frame->data[3]);
        state->report_config_valid = true;

        LOG_INF("motor report config: name=%s enable=%u period=%u ms",
                motor->name,
                state->report_enabled ? 1U : 0U,
                state->report_period_ms);
    }

    if (((frame->data[0] == GEN_MOTOR_CMD_SET_FOC_CONFIG) ||
         (frame->data[0] == GEN_MOTOR_CMD_READ_FOC_CONFIG)) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK) &&
        (frame->dlc >= 7U))
    {
        state->foc_config.param = (enum gen_motor_foc_config_param)frame->data[2];
        state->foc_config.value = sys_get_le32(&frame->data[3]);
        state->foc_config.valid = true;
    }
}

/**
 * @brief 处理管理发现上报帧。
 * @param data gen-motor 运行数据。
 * @param frame 管理发现上报帧。
 * @return void
 */
static void gen_motor_handle_management_report_locked(struct gen_motor_data *data,
                                                      const struct can_frame *frame)
{
    if (frame->dlc < 7U)
    {
        return;
    }

    if (frame->data[0] != GEN_MOTOR_CMD_MANAGEMENT_DISCOVERY)
    {
        return;
    }

    data->last_discovery.valid = true;
    data->last_discovery.configured = frame->data[1];
    data->last_discovery.node_id = frame->data[2];
    data->last_discovery.uid32 = sys_get_le32(&frame->data[3]);
}

/**
 * @brief 处理管理应答帧。
 * @param data gen-motor 运行数据。
 * @param frame 管理应答帧。
 * @return void
 */
static void gen_motor_handle_management_ack_locked(struct gen_motor_data *data,
                                                   const struct can_frame *frame)
{
    struct gen_motor *motor;
    struct gen_motor_state *state;
    uint32_t uid32;
    size_t payload_size;

    if ((data->motor_count == 0U) || (frame->dlc < 2U))
    {
        return;
    }

    if (!gen_motor_take_pending_management_ack_locked(data, frame->data[0]))
    {
        return;
    }

    motor = &data->motors[0];
    state = &motor->state;
    state->last_ack.cmd = frame->data[0];
    state->last_ack.status = frame->data[1];
    state->ack_valid = true;
    state->ack_count++;
    memset(state->last_ack.payload, 0, sizeof(state->last_ack.payload));

    if (frame->dlc > 2U)
    {
        payload_size = MIN((size_t)frame->dlc - 2U,
                           sizeof(state->last_ack.payload));
        memcpy(state->last_ack.payload, &frame->data[2], payload_size);
    }

    if ((frame->data[0] == GEN_MOTOR_CMD_MANAGEMENT_IDENTIFY) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK) &&
        (frame->dlc >= 8U))
    {
        uid32 = sys_get_le32(&frame->data[4]);
        state->uid32 = uid32;
        state->uid32_valid = true;
    }

    if ((frame->data[0] == GEN_MOTOR_CMD_MANAGEMENT_CONFIGURE_ID) &&
        (frame->data[1] == GEN_MOTOR_STATUS_OK) &&
        (frame->dlc >= 7U))
    {
        uid32 = sys_get_le32(&frame->data[3]);
        state->node_id = frame->data[2];
        state->requested_node_id = frame->data[2];
        state->reported_node_id = frame->data[2];
        state->reported_node_id_valid = true;
        state->configured = true;
        state->uid32 = uid32;
        state->uid32_valid = true;
        state->pending_node_id_change = false;
    }
}

/**
 * @brief 处理接收到的主动上报帧。
 * @param motor 电机对象。
 * @param frame 上报帧。
 * @return void
 */
static void gen_motor_handle_report_locked(struct gen_motor *motor,
                                           const struct can_frame *frame)
{
    struct gen_motor_state *state = &motor->state;

    if (frame->dlc < CAN_MAX_DLC)
    {
        return;
    }

    state->position_mrad = (int32_t)sys_get_le32(&frame->data[0]);
    state->speed_mrad_s = (int32_t)sys_get_le32(&frame->data[4]);
    state->position_valid = true;
    state->speed_valid = true;
    state->report_count++;

    if (!motor->state_report_logged)
    {
        LOG_INF("motor report: name=%s type=state", motor->name);
        motor->state_report_logged = true;
    }
}

/**
 * @brief 处理来自 CAN 传输层的接收帧。
 * @param frame 接收到的 CAN 报文。
 * @param user_data 用户参数。
 * @return void
 */
static void gen_motor_handle_can_frame(const struct can_frame *frame, void *user_data)
{
    const struct device *dev = user_data;
    struct gen_motor_data *data;
    enum gen_motor_frame_kind kind;
    struct gen_motor *motor;
    uint8_t source_node_id;

    if ((dev == NULL) || (frame == NULL))
    {
        return;
    }

    if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0U)
    {
        return;
    }

    if (!gen_motor_decode_frame_id(frame->id, &kind, &source_node_id))
    {
        return;
    }

    data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    if (!data->initialized)
    {
        k_mutex_unlock(&data->lock);
        return;
    }

    if (kind == GEN_MOTOR_FRAME_MANAGEMENT_REPORT)
    {
        gen_motor_handle_management_report_locked(data, frame);
        k_mutex_unlock(&data->lock);
        return;
    }

    if (kind == GEN_MOTOR_FRAME_MANAGEMENT_ACK)
    {
        gen_motor_handle_management_ack_locked(data, frame);
        k_mutex_unlock(&data->lock);
        return;
    }

    motor = gen_motor_find_by_any_node_locked(data, source_node_id);

    if (motor == NULL)
    {
        k_mutex_unlock(&data->lock);
        return;
    }

    motor->state.rx_count++;

    if (kind == GEN_MOTOR_FRAME_ACK)
    {
        if (frame->dlc >= 2U)
        {
            gen_motor_handle_ack_locked(motor, frame);
        }

        k_mutex_unlock(&data->lock);
        return;
    }

    if (frame->dlc >= CAN_MAX_DLC)
    {
        gen_motor_handle_report_locked(motor, frame);
    }

    k_mutex_unlock(&data->lock);
}

/**
 * @brief 注册 gen-motor 需要的 CAN 接收过滤器。
 * @param dev gen-motor 设备。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_register_filters(const struct device *dev)
{
    const struct gen_motor_config *config = dev->config;
    struct gen_motor_data *data = dev->data;
    const struct can_protocol_filter_config filters[GEN_MOTOR_FILTER_COUNT] = {
        {
            .id = GEN_MOTOR_ACK_BASE,
            .mask = GEN_MOTOR_ID_GROUP_MASK,
            .flags = 0U,
        },
        {
            .id = GEN_MOTOR_REPORT_BASE,
            .mask = GEN_MOTOR_ID_GROUP_MASK,
            .flags = 0U,
        },
    };
    size_t i;

    for (i = 0U; i < GEN_MOTOR_FILTER_COUNT; i++)
    {
        int ret;

        ret = can_protocol_add_rx_filter(config->can_protocol_dev,
                                         &filters[i],
                                         gen_motor_handle_can_frame,
                                         (void *)dev);

        if (ret < 0)
        {
            while (i > 0U)
            {
                i--;
                can_protocol_remove_rx_filter(config->can_protocol_dev,
                                              data->filter_handles[i]);
                data->filter_handles[i] = -1;
            }

            return ret;
        }

        data->filter_handles[i] = ret;
    }

    return 0;
}

/**
 * @brief 初始化 gen-motor 设备。
 * @param dev gen-motor 设备。
 * @return int 0 表示成功，负值表示失败。
 */
static int gen_motor_init(const struct device *dev)
{
    const struct gen_motor_config *config = dev->config;
    struct gen_motor_data *data = dev->data;
    size_t i;
    int ret;

    if (!device_is_ready(config->can_protocol_dev))
    {
        return -ENODEV;
    }

    data->self = dev;
    k_mutex_init(&data->lock);

    for (i = 0U; i < data->motor_count; i++)
    {
        data->motors[i].dev = dev;
        data->motors[i].index = i;
    }

    for (i = 0U; i < ARRAY_SIZE(data->filter_handles); i++)
    {
        data->filter_handles[i] = -1;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    gen_motor_reset_all_locked(data);
    k_mutex_unlock(&data->lock);

    ret = gen_motor_register_filters(dev);

    if (ret < 0)
    {
        return ret;
    }

    k_mutex_lock(&data->lock, K_FOREVER);
    data->initialized = true;
    k_mutex_unlock(&data->lock);

    LOG_DBG("gen-motor ready: motors=%u", (unsigned int)data->motor_count);

    return 0;
}

/**
 * @brief 获取默认电机对象。
 * @param dev gen-motor 设备。
 * @return const struct gen_motor * 指向默认电机对象，失败返回 NULL。
 */
const struct gen_motor *gen_motor_get_default(const struct device *dev)
{
    return gen_motor_get_by_index(dev, 0U);
}

/**
 * @brief 按索引获取电机对象。
 * @param dev gen-motor 设备。
 * @param index 电机索引。
 * @return const struct gen_motor * 找到时返回电机对象，未找到返回 NULL。
 */
const struct gen_motor *gen_motor_get_by_index(const struct device *dev, size_t index)
{
    const struct gen_motor_data *data;

    if (dev == NULL)
    {
        return NULL;
    }

    data = dev->data;

    if (index >= data->motor_count)
    {
        return NULL;
    }

    return &data->motors[index];
}

/**
 * @brief 获取电机对象数量。
 * @param dev gen-motor 设备。
 * @return size_t 已注册的电机数量。
 */
size_t gen_motor_get_count(const struct device *dev)
{
    const struct gen_motor_data *data;

    if (dev == NULL)
    {
        return 0U;
    }

    data = dev->data;
    return data->motor_count;
}

/**
 * @brief 获取最近一次未配置发现上报。
 * @param dev gen-motor 设备。
 * @param discovery 输出发现信息。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_copy_discovery(const struct device *dev, struct gen_motor_discovery *discovery)
{
    struct gen_motor_data *data;

    if ((dev == NULL) || (discovery == NULL))
    {
        return -EINVAL;
    }

    data = dev->data;
    k_mutex_lock(&data->lock, K_FOREVER);
    memcpy(discovery, &data->last_discovery, sizeof(*discovery));
    k_mutex_unlock(&data->lock);

    if (!discovery->valid)
    {
        return -ENOENT;
    }

    return 0;
}

/**
 * @brief 获取电机当前节点 ID。
 * @param motor 电机对象。
 * @param node_id 输出当前节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_get_current_node_id(const struct gen_motor *motor, uint8_t *node_id)
{
    struct gen_motor_data *data;

    if ((node_id == NULL) || !gen_motor_is_valid(motor))
    {
        return -EINVAL;
    }

    data = motor->dev->data;
    k_mutex_lock(&data->lock, K_FOREVER);
    *node_id = motor->state.node_id;
    k_mutex_unlock(&data->lock);

    return 0;
}

/**
 * @brief 拷贝指定电机的状态快照。
 * @param motor 电机对象。
 * @param state 输出状态快照。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_copy_state(const struct gen_motor *motor, struct gen_motor_state *state)
{
    struct gen_motor_data *data;

    if ((state == NULL) || !gen_motor_is_valid(motor))
    {
        return -EINVAL;
    }

    data = motor->dev->data;
    k_mutex_lock(&data->lock, K_FOREVER);
    memcpy(state, &motor->state, sizeof(*state));
    k_mutex_unlock(&data->lock);

    return 0;
}

/**
 * @brief 打印指定电机的位置和速度状态。
 * @param motor 电机对象。
 * @return void
 */
void gen_motor_log_state(const struct gen_motor *motor)
{
    struct gen_motor_state state;
    int ret;

    ret = gen_motor_copy_state(motor, &state);

    if (ret < 0)
    {
        LOG_WRN("No motor state available: %d", ret);
        return;
    }

    LOG_INF("node=0x%02x speed=%" PRId32 " pos=%" PRId32,
            state.node_id,
            state.speed_valid ? state.speed_mrad_s : 0,
            state.position_valid ? state.position_mrad : 0);
}

/**
 * @brief 设置电机运行模式和使能状态。
 * @param motor 电机对象。
 * @param enable true 表示使能，false 表示停机。
 * @param mode 控制模式。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_run_mode(const struct gen_motor *motor,
                           bool enable,
                           enum gen_motor_control_mode mode)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    bool old_enable;
    enum gen_motor_control_mode old_mode;
    int ret;

    if (mode > GEN_MOTOR_CONTROL_MODE_POSITION)
    {
        return -EINVAL;
    }

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;
    payload[0] = GEN_MOTOR_CMD_SET_RUN_MODE;
    payload[1] = enable ? 1U : 0U;
    payload[2] = (uint8_t)mode;

    k_mutex_lock(&data->lock, K_FOREVER);
    old_enable = writable_motor->state.requested_enable;
    old_mode = writable_motor->state.requested_mode;
    writable_motor->state.requested_enable = enable;
    writable_motor->state.requested_mode = mode;
    k_mutex_unlock(&data->lock);

    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.requested_enable = old_enable;
        writable_motor->state.requested_mode = old_mode;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 设置电机电流目标。
 * @param motor 电机对象。
 * @param current_target 目标电流控制量。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_current_target(const struct gen_motor *motor, int32_t current_target)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;
    payload[0] = GEN_MOTOR_CMD_SET_CURRENT_TARGET;
    sys_put_le32((uint32_t)current_target, &payload[1]);

    k_mutex_lock(&data->lock, K_FOREVER);
    old_target = writable_motor->state.requested_current_target;
    writable_motor->state.requested_current_target = current_target;
    k_mutex_unlock(&data->lock);

    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.requested_current_target = old_target;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 设置电机速度目标。
 * @param motor 电机对象。
 * @param speed_mrad_s 目标速度，单位 mrad/s。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_speed_target(const struct gen_motor *motor, int32_t speed_mrad_s)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;
    payload[0] = GEN_MOTOR_CMD_SET_SPEED_TARGET;
    sys_put_le32((uint32_t)speed_mrad_s, &payload[1]);

    k_mutex_lock(&data->lock, K_FOREVER);
    old_target = writable_motor->state.requested_speed_target_mrad_s;
    writable_motor->state.requested_speed_target_mrad_s = speed_mrad_s;
    k_mutex_unlock(&data->lock);

    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.requested_speed_target_mrad_s = old_target;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 设置电机位置目标。
 * @param motor 电机对象。
 * @param position_mrad 目标位置，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_position_target(const struct gen_motor *motor, int32_t position_mrad)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;
    payload[0] = GEN_MOTOR_CMD_SET_POSITION_TARGET;
    sys_put_le32((uint32_t)position_mrad, &payload[1]);

    k_mutex_lock(&data->lock, K_FOREVER);
    old_target = writable_motor->state.requested_position_target_mrad;
    writable_motor->state.requested_position_target_mrad = position_mrad;
    k_mutex_unlock(&data->lock);

    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.requested_position_target_mrad = old_target;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 按 UID 请求未配置电机执行识别灯效。
 * @param dev gen-motor 设备。
 * @param uid32 目标电机短 UID。
 * @param blink_hz 闪烁频率，单位 Hz。
 * @param duration_s 持续时间，单位秒，0 表示默认值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_identify_by_uid(const struct device *dev,
                              uint32_t uid32,
                              uint8_t blink_hz,
                              uint8_t duration_s)
{
    uint8_t payload[CAN_MAX_DLC] = {0};

    if (dev == NULL)
    {
        return -EINVAL;
    }

    payload[0] = GEN_MOTOR_CMD_MANAGEMENT_IDENTIFY;
    payload[1] = blink_hz;
    payload[2] = duration_s;
    gen_motor_put_uid32(payload, 3U, uid32);

    return gen_motor_send_management_payload(dev, payload);
}

/**
 * @brief 按 UID 配置未配置电机的业务节点 ID。
 * @param dev gen-motor 设备。
 * @param uid32 目标电机短 UID。
 * @param new_node_id 新业务节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_configure_by_uid(const struct device *dev,
                               uint32_t uid32,
                               uint8_t new_node_id)
{
    uint8_t payload[CAN_MAX_DLC] = {0};

    if (dev == NULL)
    {
        return -EINVAL;
    }

    if (!gen_motor_node_id_is_valid(new_node_id))
    {
        return -EINVAL;
    }

    payload[0] = GEN_MOTOR_CMD_MANAGEMENT_CONFIGURE_ID;
    payload[1] = new_node_id;
    gen_motor_put_uid32(payload, 3U, uid32);

    return gen_motor_send_management_payload(dev, payload);
}

/**
 * @brief 设置电机速度环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_speed_pid(const struct gen_motor *motor,
                            enum gen_motor_pid_param param,
                            int32_t value)
{
    if (param > GEN_MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return gen_motor_send_param_write(motor, GEN_MOTOR_CMD_SET_SPEED_PID, param, value);
}

/**
 * @brief 请求读取电机速度环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_speed_pid(const struct gen_motor *motor,
                                enum gen_motor_pid_param param)
{
    if (param > GEN_MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return gen_motor_send_u8_param_read(motor,
                                        GEN_MOTOR_CMD_READ_SPEED_PID,
                                        (uint8_t)param);
}

/**
 * @brief 设置电机位置环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_position_pid(const struct gen_motor *motor,
                               enum gen_motor_pid_param param,
                               int32_t value)
{
    if (param > GEN_MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return gen_motor_send_param_write(motor,
                                      GEN_MOTOR_CMD_SET_POSITION_PID,
                                      param,
                                      value);
}

/**
 * @brief 请求读取电机位置环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_position_pid(const struct gen_motor *motor,
                                   enum gen_motor_pid_param param)
{
    if (param > GEN_MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return gen_motor_send_u8_param_read(motor,
                                        GEN_MOTOR_CMD_READ_POSITION_PID,
                                        (uint8_t)param);
}

/**
 * @brief 请求读取电机 App 版本号。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_app_version(const struct gen_motor *motor)
{
    return gen_motor_send_simple(motor, GEN_MOTOR_CMD_READ_APP_VERSION);
}

/**
 * @brief 设置电机主动上报配置。
 * @param motor 电机对象。
 * @param enable true 表示开启主动上报，false 表示关闭。
 * @param period_ms 上报周期，单位毫秒。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_report_config(const struct gen_motor *motor,
                                bool enable,
                                uint16_t period_ms)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    bool old_enabled;
    uint16_t old_period_ms;
    int ret;

    if (!gen_motor_report_period_is_valid(period_ms))
    {
        return -EINVAL;
    }

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;
    payload[0] = GEN_MOTOR_CMD_SET_REPORT_CONFIG;
    payload[1] = enable ? 1U : 0U;
    sys_put_le16(period_ms, &payload[2]);

    k_mutex_lock(&data->lock, K_FOREVER);
    old_enabled = writable_motor->state.report_enabled;
    old_period_ms = writable_motor->state.report_period_ms;
    writable_motor->state.report_enabled = enable;
    writable_motor->state.report_period_ms = period_ms;
    k_mutex_unlock(&data->lock);

    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.report_enabled = old_enabled;
        writable_motor->state.report_period_ms = old_period_ms;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 请求读取电机主动上报配置。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_report_config(const struct gen_motor *motor)
{
    return gen_motor_send_simple(motor, GEN_MOTOR_CMD_READ_REPORT_CONFIG);
}

/**
 * @brief 设置电机 OTA 状态。
 * @param motor 电机对象。
 * @param active true 表示进入 OTA 状态，false 表示退出 OTA 状态。
 * @return int 0 表示成功，负值表示失败。
 * @note 进入 OTA 时只关闭主动上报；退出 OTA 时只恢复进入 OTA 前的主动上报状态。
 */
int gen_motor_set_ota_state(const struct gen_motor *motor, bool active)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint16_t restore_period_ms;
    bool restore_enabled;
    bool restore_valid;
    int ret;

    if (!gen_motor_is_valid(motor))
    {
        return -EINVAL;
    }

    data = motor->dev->data;
    writable_motor = &data->motors[motor->index];

    k_mutex_lock(&data->lock, K_FOREVER);

    if (active == writable_motor->state.ota_active)
    {
        k_mutex_unlock(&data->lock);
        return 0;
    }

    if (active)
    {
        writable_motor->ota_report_enabled_before_ota =
            writable_motor->state.report_enabled;
        writable_motor->ota_report_period_before_ota =
            writable_motor->state.report_period_ms;
        writable_motor->ota_report_restore_valid =
            writable_motor->state.report_config_valid;
        writable_motor->state.ota_active = true;
        restore_period_ms = writable_motor->state.report_period_ms;
        k_mutex_unlock(&data->lock);

        if (restore_period_ms < GEN_MOTOR_REPORT_PERIOD_MIN_MS)
        {
            restore_period_ms = GEN_MOTOR_REPORT_PERIOD_MIN_MS;
        }

        ret = gen_motor_set_report_config(motor, false, restore_period_ms);

        if (ret < 0)
        {
            k_mutex_lock(&data->lock, K_FOREVER);
            writable_motor->state.ota_active = false;
            writable_motor->ota_report_restore_valid = false;
            k_mutex_unlock(&data->lock);
        }

        return ret;
    }

    restore_enabled = writable_motor->ota_report_enabled_before_ota;
    restore_period_ms = writable_motor->ota_report_period_before_ota;
    restore_valid = writable_motor->ota_report_restore_valid;
    writable_motor->state.ota_active = false;
    writable_motor->ota_report_restore_valid = false;
    k_mutex_unlock(&data->lock);

    if (restore_period_ms < GEN_MOTOR_REPORT_PERIOD_MIN_MS)
    {
        restore_period_ms = GEN_MOTOR_REPORT_PERIOD_MIN_MS;
    }

    if (!restore_valid)
    {
        return 0;
    }

    ret = gen_motor_set_report_config(motor, restore_enabled, restore_period_ms);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.ota_active = true;
        writable_motor->ota_report_restore_valid = true;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 设置电机 FOC 基础配置。
 * @param motor 电机对象。
 * @param param FOC 基础配置参数编号。
 * @param value 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_foc_config(const struct gen_motor *motor,
                             enum gen_motor_foc_config_param param,
                             uint32_t value)
{
    if (!gen_motor_foc_config_param_is_valid(param))
    {
        return -EINVAL;
    }

    return gen_motor_send_u32_param_write(motor,
                                          GEN_MOTOR_CMD_SET_FOC_CONFIG,
                                          (uint8_t)param,
                                          value);
}

/**
 * @brief 请求读取电机 FOC 基础配置。
 * @param motor 电机对象。
 * @param param FOC 基础配置参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_foc_config(const struct gen_motor *motor,
                                 enum gen_motor_foc_config_param param)
{
    if (!gen_motor_foc_config_param_is_valid(param))
    {
        return -EINVAL;
    }

    return gen_motor_send_u8_param_read(motor,
                                        GEN_MOTOR_CMD_READ_FOC_CONFIG,
                                        (uint8_t)param);
}

/**
 * @brief 设置电机节点 ID。
 * @param motor 电机对象。
 * @param new_node_id 新节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_node_id(const struct gen_motor *motor, uint8_t new_node_id)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    uint8_t old_requested_node_id;
    bool old_pending;
    int ret;

    if (!gen_motor_node_id_is_valid(new_node_id))
    {
        return -EINVAL;
    }

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    data = writable_motor->dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    if (gen_motor_node_id_is_reserved_locked(data, new_node_id, writable_motor->index))
    {
        k_mutex_unlock(&data->lock);
        return -EEXIST;
    }

    old_requested_node_id = writable_motor->state.requested_node_id;
    old_pending = writable_motor->state.pending_node_id_change;
    writable_motor->state.requested_node_id = new_node_id;
    writable_motor->state.pending_node_id_change =
        (new_node_id != writable_motor->state.node_id);
    k_mutex_unlock(&data->lock);

    payload[0] = GEN_MOTOR_CMD_SET_NODE_ID;
    payload[1] = new_node_id;
    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.requested_node_id = old_requested_node_id;
        writable_motor->state.pending_node_id_change = old_pending;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 复位电机 CAN 配置信息并触发电机重启。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_reset_can_config(const struct gen_motor *motor)
{
    struct gen_motor *writable_motor;
    struct gen_motor_data *data;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = gen_motor_prepare_send(motor, &writable_motor, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = GEN_MOTOR_CMD_RESET_CAN_CONFIG;
    payload[1] = GEN_MOTOR_RESET_CAN_CONFIG_CONFIRM_CODE;
    ret = gen_motor_send_payload(writable_motor, node_id, payload);

    if (ret == 0)
    {
        data = writable_motor->dev->data;
        k_mutex_lock(&data->lock, K_FOREVER);
        writable_motor->state.configured = false;
        writable_motor->state.node_id = GEN_MOTOR_MANAGEMENT_NODE_ID;
        writable_motor->state.requested_node_id = GEN_MOTOR_MANAGEMENT_NODE_ID;
        writable_motor->state.pending_node_id_change = false;
        k_mutex_unlock(&data->lock);
    }

    return ret;
}

/**
 * @brief 请求读取电机节点 ID。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_node_id(const struct gen_motor *motor)
{
    return gen_motor_send_simple(motor, GEN_MOTOR_CMD_READ_NODE_ID);
}

/**
 * @brief 触发电机零点校准。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_zero_calibration(const struct gen_motor *motor)
{
    return gen_motor_send_simple(motor, GEN_MOTOR_CMD_ZERO_CALIBRATION);
}

/**
 * @brief 请求读取电机零点信息。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_zero(const struct gen_motor *motor)
{
    return gen_motor_send_simple(motor, GEN_MOTOR_CMD_READ_ZERO);
}

#define GEN_MOTOR_CHILD_INIT(child_id)                                                  \
    {                                                                                   \
        .name = DT_PROP(child_id, motor_name),                                          \
        .default_node_id = DT_PROP(child_id, node_id),                                  \
        .default_report_period_ms = DT_PROP(child_id, report_period_ms),                \
    },

#define GEN_MOTOR_DEFINE(inst)                                                          \
    static struct gen_motor gen_motor_motors_##inst[] = {                               \
        DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, GEN_MOTOR_CHILD_INIT)                   \
    };                                                                                  \
    static struct gen_motor_data gen_motor_data_##inst = {                              \
        .motors = gen_motor_motors_##inst,                                              \
        .motor_count = ARRAY_SIZE(gen_motor_motors_##inst),                             \
    };                                                                                  \
    static const struct gen_motor_config gen_motor_config_##inst = {                    \
        .can_protocol_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_protocol)),         \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(inst,                                                         \
                          gen_motor_init,                                               \
                          NULL,                                                         \
                          &gen_motor_data_##inst,                                       \
                          &gen_motor_config_##inst,                                     \
                          POST_KERNEL,                                                  \
                          CONFIG_HXZP_GEN_MOTOR_INIT_PRIORITY,                         \
                          NULL)

DT_INST_FOREACH_STATUS_OKAY(GEN_MOTOR_DEFINE)
