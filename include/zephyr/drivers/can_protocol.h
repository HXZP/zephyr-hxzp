/**
 * @file can_protocol.h
 * @brief HXZP CAN 协议传输层 driver 接口。
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CAN_PROTOCOL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CAN_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief CAN 接收过滤器配置。
 */
struct can_protocol_filter_config
{
    /**< 过滤器 ID。 */
    uint32_t id;
    /**< 过滤器掩码。 */
    uint32_t mask;
    /**< 过滤器标志，使用 CAN_FILTER_xxx。 */
    uint8_t flags;
};

/**
 * @brief CAN 接收回调函数类型。
 * @param frame 接收到的 CAN 报文。
 * @param user_data 用户参数。
 * @return void
 */
typedef void (*can_protocol_rx_handler_t)(const struct can_frame *frame,
                                          void *user_data);

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
                               void *user_data);

/**
 * @brief 从 CAN 协议传输层移除接收过滤器。
 * @param dev CAN 协议传输层设备。
 * @param filter_handle 过滤器句柄。
 * @return void
 */
void can_protocol_remove_rx_filter(const struct device *dev, int filter_handle);

/**
 * @brief 将 CAN 报文放入发送队列。
 * @param dev CAN 协议传输层设备。
 * @param frame 待发送的 CAN 报文。
 * @return int 0 表示成功放入队列，负值表示失败。
 */
int can_protocol_send(const struct device *dev, const struct can_frame *frame);

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
                           uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CAN_PROTOCOL_H_ */
