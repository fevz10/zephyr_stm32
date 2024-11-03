#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>

#define SLEEP_TIME_MS   1000

#define LED0_NODE DT_ALIAS(led0)
#define ADC_CHANNEL_0_SOC DT_PATH(adc1, ch0)
#define ADC_CHANNEL_1_VOLT DT_PATH(adc1, ch1)
#define ADC_RESOLUTION 12

static const struct device *adc_dev;
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;
    printk("Starting Zephyr project\n");
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

    if (!device_is_ready(adc_dev))
    {
        printk("ADC device not found\n");
        return -1;
    }
    
    struct adc_channel_cfg adc_cfg_0 = 
    {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 0,
        .differential = 0,
    };

    adc_channel_setup(adc_dev, &adc_cfg_0);

    struct adc_channel_cfg adc_cfg_1 =
    {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 1,
        .differential = 0,
    };

    adc_channel_setup(adc_dev, &adc_cfg_1);
	ret = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE);
    printk("Devices initialized\n");

	while (1) 
	{
        struct adc_sequence sequence;
        int16_t sample;

        sequence.channels = BIT(0);
        sequence.buffer = &sample;
        sequence.buffer_size = sizeof(sample);
        sequence.resolution = ADC_RESOLUTION;

        adc_read(adc_dev, &sequence);
        printk("ADC Channel 0: %d\n", sample);

        sequence.channels = BIT(1);
        adc_read(adc_dev, &sequence);
        printk("ADC Channel 1: %d\n", sample);

		ret = gpio_pin_toggle_dt(&led_green);
		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");

		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}