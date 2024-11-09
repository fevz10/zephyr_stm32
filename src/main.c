#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zephyr_stm32);

#define ADC_THREAD_STACK_SIZE 1024
#define ADC_THREAD_PRIORITY 7
#define CAN_THREAD_STACK_SIZE 1024
#define CAN_THREAD_PRIORITY 7

#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL

#define ADC_CHANNEL_ID_0 0
#define ADC_CHANNEL_ID_1 1

#define CANBUS_NODE DT_CHOSEN(zephyr_canbus)
#define LED0_NODE DT_ALIAS(led0)
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)


K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_thread_stack, CAN_THREAD_STACK_SIZE);

struct gpio_dt_spec buttonSoC = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
struct gpio_dt_spec buttonVolt = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//static const struct device *gpio_dev;
static int16_t adc_sample_buffer1[1], adc_sample_buffer2[1];

struct k_thread adc_thread_data;
struct k_thread can_thread_data;

struct can_frame can_frame = {
	.flags = 0,
	.id = 0x123,
	.dlc = 8
};
int btnStateSoC, btnStateVolt;

void adc_read_channels();
void can_send_message();
void adc_task_handler(void *arg1, void *arg2, void *arg3);
void can_task_handler(void *arg1, void *arg2, void *arg3);


int main(void)
{
    int ret;
	k_tid_t adc_tid, can_tid;
    printf("Starting Zephyr Project\n");

    if(!device_is_ready(adc_dev))
    {
        printf("ADC: Device %s not ready.\n", adc_dev->name);
    }

	printf("ADC: Device %s is initialized.\n", adc_dev->name);

    if(!device_is_ready(can_dev))
    {
        printf("CAN: Device %s not ready.\n", can_dev->name);
    }

    ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
    if (0 != ret)
    {
		printf("Error setting CAN mode [%d]\n", ret);
		return -1;  
    }
    ret = can_start(can_dev);
	if (0 != ret)
	{
		printf("Error starting CAN controller [%d]\n", ret);
		return 0;
	}

    printf("CAN: Device %s is initialized.\n", can_dev->name);

    if (buttonSoC.port != NULL)
    {
		if (!gpio_is_ready_dt(&buttonSoC))
		{
			printf("SW: Device is not ready.\n");
			return -1;
		}
		ret = gpio_pin_configure_dt(&buttonSoC, GPIO_INPUT);
		if (ret < 0)
		{
			printf("Error setting SW pin to input mode [%d]", ret);
			buttonSoC.port = NULL;
		}
    }

    if (buttonVolt.port != NULL)
    {
		if (!gpio_is_ready_dt(&buttonVolt))
		{
			printf("SW: Device is not ready.\n");
			return -1;
		}
		ret = gpio_pin_configure_dt(&buttonVolt, GPIO_INPUT);
		if (ret < 0)
		{
			printf("Error setting SW pin to input mode [%d]", ret);
			buttonVolt.port = NULL;
		}
    }

	if (led.port != NULL)
	{
		if (!gpio_is_ready_dt(&led))
		{
			printf("LED: Device %s not ready.\n", led.port->name);
			return -1;
		}
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
		if (ret < 0)
		{
			printf("Error setting LED pin to output mode [%d]", ret);
			led.port = NULL;
		}
	}

	printf("Device initializations are finished\n");
	
	adc_tid = k_thread_create(
		&adc_thread_data,
		adc_thread_stack,
		K_THREAD_STACK_SIZEOF(adc_thread_stack),
		adc_task_handler,
		NULL,NULL,NULL,
		ADC_THREAD_PRIORITY, 0,K_NO_WAIT);
	if (!adc_tid)
	{
		printf("ERROR spawning ADC THREAD\n");
	}
	

	can_tid = k_thread_create(
		&can_thread_data,
		can_thread_stack,
		K_THREAD_STACK_SIZEOF(can_thread_stack),
		can_task_handler,
		NULL,NULL,NULL,
		CAN_THREAD_PRIORITY, 0,K_NO_WAIT);
	if (!can_tid)
	{
		printf("ERROR spawning CAN THREAD\n");
	}

	printf("Starting Zephyr App\n");
}


void adc_read_channels()
{
    struct adc_sequence sequence_1 = {
        .channels = BIT(ADC_CHANNEL_ID_0),
        .buffer = adc_sample_buffer1,
        .buffer_size = sizeof(adc_sample_buffer1),
        .resolution = ADC_RESOLUTION,
    };
    struct adc_sequence sequence_2 = {
        .channels = BIT(ADC_CHANNEL_ID_1),
        .buffer = adc_sample_buffer2,
        .buffer_size = sizeof(adc_sample_buffer2),
        .resolution = ADC_RESOLUTION,
    };


    if (adc_read(adc_dev, &sequence_1) != 0)
    {
        printf("ADC read failed\n");
    }
	if (adc_read(adc_dev, &sequence_2) != 0)
    {
        printf("ADC read failed\n");
    } 
    btnStateSoC = gpio_pin_get_dt(&buttonSoC);
    btnStateVolt = gpio_pin_get_dt(&buttonVolt);
    //printf("SW0 State : %d\n", btnStateSoC);
    //printf("SW1 State : %d\n", btnStateVolt);
    
    printf("ADC readings: %d, %d\nDigital Input readings : %d, %d\n",
                adc_sample_buffer1[0], adc_sample_buffer2[0],
                btnStateSoC, btnStateVolt);
    
}

void can_send_message()
{
    can_frame.data[0] = adc_sample_buffer1[0] & 0xFF;
    can_frame.data[1] = (adc_sample_buffer1[0] >> 8) & 0xFF;
    can_frame.data[2] = adc_sample_buffer2[0] & 0xFF;
    can_frame.data[3] = (adc_sample_buffer2[0] >> 8) & 0xFF;
    can_frame.data[4] = btnStateSoC & 0xFF;
    can_frame.data[5] = btnStateVolt & 0xFF;
    can_frame.data[6] = 0x00;
    can_frame.data[7] = 0x00;

    if (can_send(can_dev, &can_frame, K_MSEC(100), NULL, NULL) != 0)
    {
        printf("CAN send failed\n");
        gpio_pin_set_dt(&led, 0);
    } 
    else
    {
        //printf("CAN message sent\n");
        gpio_pin_toggle_dt(&led);
        //gpio_pin_set_dt(&led, 1);
    }
}

void adc_task_handler(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
    while (1)
	{
        adc_read_channels();
        k_sleep(K_MSEC(25));
    }
}

void can_task_handler(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
    while (1) 
	{
        can_send_message();
        k_sleep(K_MSEC(100));
    }
}