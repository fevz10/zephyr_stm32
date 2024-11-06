/*
 * Copyright (c) 2018 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

/*
#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
*/
#define LED_MSG_ID 0xFF
#define SET_LED 1
#define RESET_LED 0
#define SLEEP_TIME K_MSEC(250)

#define LED0_NODE DT_ALIAS(led0)

#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_CHANNEL_ID_1 0 // First channel (e.g., PA0)
#define ADC_CHANNEL_ID_2 1 // Second channel (e.g., PA1)
#define ADC_BUFFER_SIZE 1  // Buffer size for two channels

static const struct adc_channel_cfg channel_cfg_1 = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL_ID_1,
    .differential = 0,
};

static const struct adc_channel_cfg channel_cfg_2 = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL_ID_2,
    .differential = 0,
};

static int16_t sample_buffer_1[ADC_BUFFER_SIZE];
static int16_t sample_buffer_2[ADC_BUFFER_SIZE];

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


struct k_work_poll change_led_work;
struct k_work state_change_work;
enum can_state current_state;
struct can_bus_err_cnt current_err_cnt;

CAN_MSGQ_DEFINE(change_led_msgq, 2);

static struct k_poll_event change_led_events[1] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&change_led_msgq, 0)
};

void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	char *sender = (char *)arg;

	ARG_UNUSED(dev);

	if (error != 0) {
		printf("Callback! error-code: %d\nSender: %s\n",
		       error, sender);
	}
}

char *state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

void state_change_work_handler(struct k_work *work)
{
	printf("State Change ISR\nstate: %s\n"
	       "rx error count: %d\n"
	       "tx error count: %d\n",
		state_to_str(current_state),
		current_err_cnt.rx_err_cnt, current_err_cnt.tx_err_cnt);
}

void state_change_callback(const struct device *dev, enum can_state state,
			   struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct k_work *work = (struct k_work *)user_data;

	ARG_UNUSED(dev);

	current_state = state;
	current_err_cnt = err_cnt;
	k_work_submit(work);
}

int main(void)
{
	printf("Starting Zephyr project\n");
    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

    if (!device_is_ready(adc_dev)) {
        printf("ADC device not ready");
        return -1;
    }

    if (adc_channel_setup(adc_dev, &channel_cfg_1) < 0) {
        printf("Error setting up channel 1");
        return -1;
    }

    if (adc_channel_setup(adc_dev, &channel_cfg_2) < 0) {
        printf("Error setting up channel 2");
        return -1;
    }

    struct adc_sequence sequence_1 = {
        .channels    = BIT(ADC_CHANNEL_ID_1),
        .buffer      = sample_buffer_1,
        .buffer_size = sizeof(sample_buffer_1),
        .resolution  = ADC_RESOLUTION,
    };

	struct adc_sequence sequence_2 = {
        .channels    = BIT(ADC_CHANNEL_ID_2),
        .buffer      = sample_buffer_2,
        .buffer_size = sizeof(sample_buffer_2),
        .resolution  = ADC_RESOLUTION,
    };
	printf("ADC device is initialized\n");

	const struct can_filter change_led_filter = {
		.flags = 0U,
		.id = LED_MSG_ID,
		.mask = CAN_STD_ID_MASK
	};
	struct can_frame change_led_frame = {
		.flags = 0,
		.id = LED_MSG_ID,
		.dlc = 1
	};
	uint8_t toggle = 1;
	//uint16_t counter = 0;
	int ret;

	if (!device_is_ready(can_dev))
	{
		printf("CAN: Device %s not ready.\n", can_dev->name);
		return 0;
	}
	printf("CAN device is initialized\n");

	ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (ret != 0) {
	printf("Error setting CAN mode [%d]", ret);
	return 0;
	}
	ret = can_start(can_dev);
	if (ret != 0) {
		printf("Error starting CAN controller [%d]", ret);
		return 0;
	}

	if (led.port != NULL) {
		if (!gpio_is_ready_dt(&led)) {
			printf("LED: Device %s not ready.\n",
			       led.port->name);
			return 0;
		}
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
		if (ret < 0) {
			printf("Error setting LED pin to output mode [%d]",
			       ret);
			led.port = NULL;
		}
	}

	k_work_init(&state_change_work, state_change_work_handler);
	//k_work_poll_init(&change_led_work, change_led_work_handler);

	ret = can_add_rx_filter_msgq(can_dev, &change_led_msgq, &change_led_filter);
	if (ret == -ENOSPC) {
		printf("Error, no filter available!\n");
		return 0;
	}

	printf("Change LED filter ID: %d\n", ret);

	ret = k_work_poll_submit(&change_led_work, change_led_events,
				 ARRAY_SIZE(change_led_events), K_FOREVER);
	if (ret != 0) {
		printf("Failed to submit msgq polling: %d", ret);
		return 0;
	}

	/*
	rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 rx_thread, NULL, NULL, NULL,
				 RX_THREAD_PRIORITY, 0, K_NO_WAIT);
	if (!rx_tid) {
		printf("ERROR spawning rx thread\n");
	}

	get_state_tid = k_thread_create(&poll_state_thread_data,
					poll_state_stack,
					K_THREAD_STACK_SIZEOF(poll_state_stack),
					poll_state_thread, NULL, NULL, NULL,
					STATE_POLL_THREAD_PRIORITY, 0,
					K_NO_WAIT);
	if (!get_state_tid) {
		printf("ERROR spawning poll_state_thread\n");
	}
	*/

	can_set_state_change_callback(can_dev, state_change_callback, &state_change_work);

	printf("Finished init.\n");

	while (1)
	{
        if ((adc_read(adc_dev, &sequence_1) < 0))
		{
            printf("Error reading ADC\n");
        } else {
            //printf("ADC Channel 0: %d\n", sample_buffer_1[0]);
        }

		if (adc_read(adc_dev, &sequence_2) < 0)
		{
            printf("Error reading ADC\n");
        } else {
            //printf("ADC Channel 1: %d\n", sample_buffer_2[0]);
        }

		printf("ADC Channel 0: %d	|	ADC Channel 1: %d\n", sample_buffer_1[0], sample_buffer_2[0]);

		change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;
		/* This sending call is none blocking. */
		can_send(can_dev, &change_led_frame, K_FOREVER,
			 tx_irq_callback,
			 "LED change");
        ret = gpio_pin_toggle_dt(&led);
		//UNALIGNED_PUT(sys_cpu_to_be16(counter),(uint16_t *)&counter_frame.data[0]);
		//counter++;
		/* This sending call is blocking until the message is sent. */
		//can_send(can_dev, &counter_frame, K_MSEC(100), NULL, NULL);
		//printf("CAN Send Frame success!\n");
		k_sleep(SLEEP_TIME);
	}
}
