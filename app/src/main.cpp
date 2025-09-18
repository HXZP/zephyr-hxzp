#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
// #if DT_NODE_EXISTS(DT_NODELABEL(button1))
// const struct device *led_dev = DEVICE_DT_GET(DT_NODELABEL(led1));
// #else
// #error "LED1 node not defined in device tree"
// #endif
//const struct device *led_dev = DEVICE_DT_GET(DT_NODELABEL(led1));

#define LED1_NODE DT_NODELABEL(led1)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

int main() {

    // if (!device_is_ready(led_dev))
    // {
    //     return -ENODEV;
    // }
    // led_blink(led_dev, 0, 500, 500);

    // 检查设备是否就绪
    if (!gpio_is_ready_dt(&led1))
    {
        printk("LED device is not ready\n");
        return -ENODEV;
    }

    // 配置GPIO引脚
    int ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        printk("Failed to configure LED: %d\n", ret);
        return ret;
    }

    printk("main thread started.\n");

    while (true) {
        gpio_pin_toggle_dt(&led1);
        printk("main thread wakeup.\n");
        k_msleep(500);
    }
}
