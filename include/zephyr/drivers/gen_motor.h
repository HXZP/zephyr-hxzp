/**
 * @file gen_motor.h
 * @brief HXZP gen-motor driver 接口。
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_GEN_MOTOR_H_
#define ZEPHYR_INCLUDE_DRIVERS_GEN_MOTOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 电机 PID 增益参数缩放倍数。 */
#define GEN_MOTOR_PID_GAIN_SCALE 1000

/**
 * @brief 电机控制模式定义。
 */
enum gen_motor_control_mode
{
    /**< 电流控制模式。 */
    GEN_MOTOR_CONTROL_MODE_CURRENT = 0,
    /**< 速度控制模式。 */
    GEN_MOTOR_CONTROL_MODE_SPEED = 1,
    /**< 位置控制模式。 */
    GEN_MOTOR_CONTROL_MODE_POSITION = 2,
};

/**
 * @brief 电机 PID 参数编号定义。
 */
enum gen_motor_pid_param
{
    /**< 比例参数 P。 */
    GEN_MOTOR_PID_P = 0,
    /**< 积分参数 I。 */
    GEN_MOTOR_PID_I = 1,
    /**< 微分参数 D。 */
    GEN_MOTOR_PID_D = 2,
    /**< 积分累计上限。 */
    GEN_MOTOR_PID_I_ACC_MAX = 3,
    /**< 输出限幅。 */
    GEN_MOTOR_PID_OUT_MAX = 4,
};

/**
 * @brief 电机 FOC 基础配置参数编号定义。
 */
enum gen_motor_foc_config_param
{
    /**< 电机极对数。 */
    GEN_MOTOR_FOC_CONFIG_POLE_PAIRS = 0,
    /**< 主电源电压，单位毫伏。 */
    GEN_MOTOR_FOC_CONFIG_MASTER_VOLTAGE_MV = 1,
    /**< 控制环频率，单位 Hz。 */
    GEN_MOTOR_FOC_CONFIG_CONTROL_HZ = 2,
    /**< 传感器采样频率，单位 Hz。 */
    GEN_MOTOR_FOC_CONFIG_SENSOR_HZ = 3,
    /**< FOC 三相映射编号。 */
    GEN_MOTOR_FOC_CONFIG_PHASE_MAP = 4,
};

/**
 * @brief 电机对象前向声明。
 */
struct gen_motor;

/**
 * @brief 电机最近一次应答帧内容。
 */
struct gen_motor_ack
{
    /**< 应答帧中的命令码。 */
    uint8_t cmd;
    /**< 应答状态码。 */
    uint8_t status;
    /**< 附加返回数据。 */
    uint8_t payload[6];
};

/**
 * @brief 电机管理发现信息。
 */
struct gen_motor_discovery
{
    /**< 是否已经收到有效发现上报。 */
    bool valid;
    /**< 电机配置状态，0 表示未配置，1 表示已配置。 */
    uint8_t configured;
    /**< 当前节点 ID。 */
    uint8_t node_id;
    /**< 电机短 UID。 */
    uint32_t uid32;
};

/**
 * @brief 电机 App 版本信息。
 */
struct gen_motor_version
{
    /**< 主版本号。 */
    uint8_t major;
    /**< 次版本号。 */
    uint8_t minor;
    /**< 修订版本号。 */
    uint8_t patch;
};

/**
 * @brief 电机 FOC 基础配置缓存。
 */
struct gen_motor_foc_config
{
    /**< 最近一次读回的参数编号。 */
    enum gen_motor_foc_config_param param;
    /**< 最近一次读回的参数值。 */
    uint32_t value;
    /**< 读回值是否有效。 */
    bool valid;
};

/**
 * @brief 电机状态快照。
 */
struct gen_motor_state
{
    /**< 当前生效的节点 ID。 */
    uint8_t node_id;
    /**< 该对象是否使用已配置业务节点。 */
    bool configured;
    /**< 最近一次关联到的电机短 UID。 */
    uint32_t uid32;
    /**< 电机短 UID 是否有效。 */
    bool uid32_valid;
    /**< 最近一次请求设置的节点 ID。 */
    uint8_t requested_node_id;
    /**< 通过读节点 ID 命令获取到的节点 ID。 */
    uint8_t reported_node_id;
    /**< 读取到的节点 ID 是否有效。 */
    bool reported_node_id_valid;
    /**< 是否存在待完成的节点 ID 切换。 */
    bool pending_node_id_change;
    /**< 最近一次请求的使能状态。 */
    bool requested_enable;
    /**< 最近一次请求的电流目标。 */
    int32_t requested_current_target;
    /**< 最近一次请求的控制模式。 */
    enum gen_motor_control_mode requested_mode;
    /**< 最近一次请求的速度目标。 */
    int32_t requested_speed_target_mrad_s;
    /**< 最近一次请求的位置目标。 */
    int32_t requested_position_target_mrad;
    /**< 最近一次接收到的位置上报值。 */
    int32_t position_mrad;
    /**< 最近一次接收到的速度上报值。 */
    int32_t speed_mrad_s;
    /**< 最近一次读回的 App 版本。 */
    struct gen_motor_version app_version;
    /**< App 版本是否有效。 */
    bool app_version_valid;
    /**< 最近一次读回的上报使能状态。 */
    bool report_enabled;
    /**< 最近一次读回的上报周期，单位毫秒。 */
    uint16_t report_period_ms;
    /**< 上报配置是否有效。 */
    bool report_config_valid;
    /**< 最近一次读回的 FOC 基础配置。 */
    struct gen_motor_foc_config foc_config;
    /**< 位置上报是否有效。 */
    bool position_valid;
    /**< 速度上报是否有效。 */
    bool speed_valid;
    /**< 最近一次应答帧内容。 */
    struct gen_motor_ack last_ack;
    /**< 最近一次应答帧是否有效。 */
    bool ack_valid;
    /**< 最近一次 CAN 错误码。 */
    int last_can_error;
    /**< 已发送报文数量。 */
    uint32_t tx_count;
    /**< 已接收报文数量。 */
    uint32_t rx_count;
    /**< 已解析应答帧数量。 */
    uint32_t ack_count;
    /**< 已解析上报帧数量。 */
    uint32_t report_count;
    /**< 当前是否标记为 OTA 状态。 */
    bool ota_active;
};

/**
 * @brief 获取默认电机对象。
 * @param dev gen-motor 设备。
 * @return const struct gen_motor * 指向默认电机对象，失败返回 NULL。
 */
const struct gen_motor *gen_motor_get_default(const struct device *dev);

/**
 * @brief 按索引获取电机对象。
 * @param dev gen-motor 设备。
 * @param index 电机索引。
 * @return const struct gen_motor * 找到时返回电机对象，未找到返回 NULL。
 */
const struct gen_motor *gen_motor_get_by_index(const struct device *dev, size_t index);

/**
 * @brief 获取电机对象数量。
 * @param dev gen-motor 设备。
 * @return size_t 已注册的电机数量。
 */
size_t gen_motor_get_count(const struct device *dev);

/**
 * @brief 获取最近一次未配置发现上报。
 * @param dev gen-motor 设备。
 * @param discovery 输出发现信息。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_copy_discovery(const struct device *dev, struct gen_motor_discovery *discovery);

/**
 * @brief 获取电机当前节点 ID。
 * @param motor 电机对象。
 * @param node_id 输出当前节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_get_current_node_id(const struct gen_motor *motor, uint8_t *node_id);

/**
 * @brief 拷贝指定电机的状态快照。
 * @param motor 电机对象。
 * @param state 输出状态快照。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_copy_state(const struct gen_motor *motor, struct gen_motor_state *state);

/**
 * @brief 打印指定电机的位置和速度状态。
 * @param motor 电机对象。
 * @return void
 */
void gen_motor_log_state(const struct gen_motor *motor);

/**
 * @brief 设置电机运行模式和使能状态。
 * @param motor 电机对象。
 * @param enable true 表示使能，false 表示停机。
 * @param mode 控制模式。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_run_mode(const struct gen_motor *motor,
                           bool enable,
                           enum gen_motor_control_mode mode);

/**
 * @brief 设置电机电流目标。
 * @param motor 电机对象。
 * @param current_target 目标电流控制量。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_current_target(const struct gen_motor *motor, int32_t current_target);

/**
 * @brief 设置电机速度目标。
 * @param motor 电机对象。
 * @param speed_mrad_s 目标速度，单位 mrad/s。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_speed_target(const struct gen_motor *motor, int32_t speed_mrad_s);

/**
 * @brief 设置电机位置目标。
 * @param motor 电机对象。
 * @param position_mrad 目标位置，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_position_target(const struct gen_motor *motor, int32_t position_mrad);

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
                              uint8_t duration_s);

/**
 * @brief 按 UID 配置未配置电机的业务节点 ID。
 * @param dev gen-motor 设备。
 * @param uid32 目标电机短 UID。
 * @param new_node_id 新业务节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_configure_by_uid(const struct device *dev,
                               uint32_t uid32,
                               uint8_t new_node_id);

/**
 * @brief 设置电机速度环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_speed_pid(const struct gen_motor *motor,
                            enum gen_motor_pid_param param,
                            int32_t value);

/**
 * @brief 请求读取电机速度环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_speed_pid(const struct gen_motor *motor,
                                enum gen_motor_pid_param param);

/**
 * @brief 设置电机位置环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_position_pid(const struct gen_motor *motor,
                               enum gen_motor_pid_param param,
                               int32_t value);

/**
 * @brief 请求读取电机位置环 PID 参数。
 * @param motor 电机对象。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_position_pid(const struct gen_motor *motor,
                                   enum gen_motor_pid_param param);

/**
 * @brief 请求读取电机 App 版本号。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_app_version(const struct gen_motor *motor);

/**
 * @brief 设置电机主动上报配置。
 * @param motor 电机对象。
 * @param enable true 表示开启主动上报，false 表示关闭。
 * @param period_ms 上报周期，单位毫秒。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_report_config(const struct gen_motor *motor,
                                bool enable,
                                uint16_t period_ms);

/**
 * @brief 请求读取电机主动上报配置。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_report_config(const struct gen_motor *motor);

/**
 * @brief 设置电机 OTA 状态。
 * @param motor 电机对象。
 * @param active true 表示进入 OTA 状态，false 表示退出 OTA 状态。
 * @return int 0 表示成功，负值表示失败。
 * @note 进入 OTA 时只关闭主动上报；退出 OTA 时只恢复进入 OTA 前的主动上报状态。
 */
int gen_motor_set_ota_state(const struct gen_motor *motor, bool active);

/**
 * @brief 设置电机 FOC 基础配置。
 * @param motor 电机对象。
 * @param param FOC 基础配置参数编号。
 * @param value 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_foc_config(const struct gen_motor *motor,
                             enum gen_motor_foc_config_param param,
                             uint32_t value);

/**
 * @brief 请求读取电机 FOC 基础配置。
 * @param motor 电机对象。
 * @param param FOC 基础配置参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_foc_config(const struct gen_motor *motor,
                                 enum gen_motor_foc_config_param param);

/**
 * @brief 设置电机节点 ID。
 * @param motor 电机对象。
 * @param new_node_id 新节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_set_node_id(const struct gen_motor *motor, uint8_t new_node_id);

/**
 * @brief 复位电机 CAN 配置信息并触发电机重启。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_reset_can_config(const struct gen_motor *motor);

/**
 * @brief 请求读取电机节点 ID。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_node_id(const struct gen_motor *motor);

/**
 * @brief 触发电机零点校准。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_zero_calibration(const struct gen_motor *motor);

/**
 * @brief 请求读取电机零点信息。
 * @param motor 电机对象。
 * @return int 0 表示成功，负值表示失败。
 */
int gen_motor_request_zero(const struct gen_motor *motor);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_GEN_MOTOR_H_ */
