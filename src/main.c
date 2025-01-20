#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/bindesc.h>
#include "MFRC522.h"

LOG_MODULE_REGISTER(zephyr_stm32);
BINDESC_STR_DEFINE(app_version, BINDESC_ID_APP_VERSION_STRING, "1.0.0");

#define ADC_THREAD_STACK_SIZE 1024
#define ADC_THREAD_PRIORITY 7
#define CAN_THREAD_STACK_SIZE 1024
#define CAN_THREAD_PRIORITY 7
#define MFRC522_THREAD_STACK_SIZE 1024
#define MFRC522_THREAD_PRIORITY 7

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
K_THREAD_STACK_DEFINE(mfrc522_thread_stack, MFRC522_THREAD_STACK_SIZE);

struct gpio_dt_spec buttonSoC = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
struct gpio_dt_spec buttonVolt = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct device *can_dev = DEVICE_DT_GET(CANBUS_NODE);
struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//static const struct device *gpio_dev;
static int16_t adc_sample_buffer1[1], adc_sample_buffer2[1];

struct k_thread adc_thread_data;
struct k_thread can_thread_data;
struct k_thread mfrc522_thread_data;

struct EVSE_voltAmp {
    float presentVoltage;
    float presentCurrent;
	float targetVoltage;
	float targetCurrent;
}evse_voltAmp;

struct EVSE_status {
	uint8_t evseState;
	uint8_t evse_session_result;
	uint8_t err_no_available_power_module:1;
	uint8_t err_power_module:1;
	uint8_t err_contactor:1;
	uint8_t err_PLC_box_rx_timeout:1;
	uint8_t err_current_sensor_rx_timeout:1;
	uint8_t err_CP:1;
	uint8_t CPstate:3;
}evse_status;

struct EVSE_whTime {
	uint32_t evseElapsedChargeTime;
	float evseDeliveredEnergy;
}evse_whTime;

struct SECC_MSG4 {
	uint8_t EVReady:1;
	uint8_t EVCabinConditioning:1;
	uint8_t EVRESSConditioning:1;
	uint8_t BulkChargingComplete:1;
	uint8_t ChargingComplete:1;
	uint8_t EVErrorCode;
	uint16_t EVStatus_EVRESSSOC;
}secc_msg4;


uint8_t can_data_600[8];
uint8_t can_data_601[8];
uint8_t can_data_602[8];
uint8_t can_data_15110007[8];
uint8_t can_data_DEADBEEF[8];


struct can_frame can_frame_600 = {
	.flags = 0,
	.id = 0x600,
	.dlc = 4
};
struct can_frame can_frame_601 = {
	.flags = 0,
	.id = 0x601,
	.dlc = 4
};
struct can_frame can_frame_602 = {
	.flags = 0,
	.id = 0x602,
	.dlc = 7
};
struct can_frame can_frame_15110007 = {
	.flags = CAN_FRAME_IDE,
	.id = 0x15110007,
	.dlc = 3,
};
struct can_frame can_frame_DEADBEEF = {
	.flags = CAN_FRAME_IDE,
	.id = 0x1,
	.dlc = 5,
};

int btnStatePlug, btnStateCharge;
uint8_t status;
uint8_t UUID[MAX_LEN];
uint8_t detectedUUID[MAX_LEN];
bool isRFIDCardDetected = false;

void adc_read_channels();
void can_send_message();
void adc_task_handler(void *arg1, void *arg2, void *arg3);
void can_task_handler(void *arg1, void *arg2, void *arg3);
void mfrc522_task_handler(void *arg1, void *arg2, void *arg3);
void encode_EVSE_voltAmp(uint8_t *canBuffer, struct EVSE_voltAmp *voltAmp);
void encode_EVSE_status(uint8_t *canBuffer, struct EVSE_status *status);
void encode_EVSE_whTime(uint8_t *canBuffer, struct EVSE_whTime *whTime);
void encode_SECC_MSG4(uint8_t *canBuffer, struct SECC_MSG4 *msg4);
void writeCanBits(uint64_t *canMsg, int startBit, int bitLen, uint64_t bits);
void writeCanParam(uint64_t *canMsg, int startBit, int bitLen, float param, float resl, float ofset);
void can64toCanBytes(uint64_t can64, uint8_t *canBytes);
long map(long x, long in_min, long in_max, long out_min, long out_max);

int main(void)
{
    int ret;
	k_tid_t adc_tid, can_tid, mfrc522_tid;
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

	SPI_Init();
	MFRC522_Init();

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

	mfrc522_tid = k_thread_create(
		&mfrc522_thread_data,
		mfrc522_thread_stack,
		K_THREAD_STACK_SIZEOF(mfrc522_thread_stack),
		mfrc522_task_handler,
		NULL,NULL,NULL,
		MFRC522_THREAD_PRIORITY, 0,K_NO_WAIT);
	if (!mfrc522_tid)
	{
		printf("ERROR spawning MFRC522 THREAD\n");
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


    if (0 != adc_read(adc_dev, &sequence_1))
    {
        printf("ADC read failed\n");
    }
	if (0 != adc_read(adc_dev, &sequence_2))
    {
        printf("ADC read failed\n");
    } 
    btnStateCharge = gpio_pin_get_dt(&buttonSoC);
    btnStatePlug = gpio_pin_get_dt(&buttonVolt);

    /*
    printf("ADC readings: %d, %d\nDigital Input readings : %d, %d\n",
                adc_sample_buffer1[0], adc_sample_buffer2[0],
                btnStateCharge, btnStatePlug);
	*/
    
}

void can_send_message()
{
	adc_sample_buffer1[0] = map(adc_sample_buffer1[0], 0, 4095, 0, 4000);
	int16_t cnst_current = 11000;

	can_frame_600.data[0] = (btnStateCharge == 0x01) ? 0x0E : 0x07;
	can_frame_600.data[3] = ((btnStatePlug == 0x01) ? 0x02 : 0x01) << 3;
    can_frame_601.data[0] = adc_sample_buffer1[0] & 0xFF;
    can_frame_601.data[1] = (adc_sample_buffer1[0] >> 8) & 0xFF;
    can_frame_601.data[2] = cnst_current & 0xFF;
    can_frame_601.data[3] = (cnst_current >> 8) & 0xFF;
	can_frame_15110007.data[2] = map(adc_sample_buffer2[0], 0, 4095, 0, 100);

	if (true == isRFIDCardDetected)
	{	
		can_frame_DEADBEEF.data[0] = UUID[0];
		can_frame_DEADBEEF.data[1] = UUID[1];
		can_frame_DEADBEEF.data[2] = UUID[2];
		can_frame_DEADBEEF.data[3] = UUID[3];
		can_frame_DEADBEEF.data[4] = UUID[4];
	}
	else if(false == isRFIDCardDetected)
	{
		can_frame_DEADBEEF.data[0] = 0x00;
		can_frame_DEADBEEF.data[1] = 0x00;
		can_frame_DEADBEEF.data[2] = 0x00;
		can_frame_DEADBEEF.data[3] = 0x00;
		can_frame_DEADBEEF.data[4] = 0x00;
	}

	//encode_EVSE_status(can_frame_600.data, &evse_status);
	//encode_EVSE_voltAmp(can_frame_601.data, &evse_voltAmp);
	//encode_EVSE_whTime(can_frame_602.data, &evse_whTime);
	//encode_SECC_MSG4(can_frame_15110007.data, &secc_msg4);

	/*
	memcpy(can_frame_600.data, can_data_600, 8 * sizeof(can_data_600[0]));
	memcpy(can_frame_601.data, can_data_601, 8 * sizeof(can_data_601[0]));
	memcpy(can_frame_602.data, can_data_602, 8 * sizeof(can_data_602[0]));
	memcpy(can_frame_15110007.data, can_data_15110007, 8 * sizeof(can_data_15110007[0]));
	*/
	can_send(can_dev, &can_frame_600, K_MSEC(100), NULL, NULL);
	can_send(can_dev, &can_frame_601, K_MSEC(100), NULL, NULL);
	can_send(can_dev, &can_frame_602, K_MSEC(100), NULL, NULL);
	can_send(can_dev, &can_frame_15110007, K_MSEC(100), NULL, NULL);
	can_send(can_dev, &can_frame_DEADBEEF, K_MSEC(100), NULL, NULL);
	/*
	can_frame_600.data[0] = (uint8_t)btnStateCharge;
	can_frame_600.data[3] =  (uint8_t)btnStatePlug;

	can_frame_601.data[0] = adc_sample_buffer1[0] & 0xFF;
	can_frame_601.data[1] = (adc_sample_buffer1[0] >> 8) & 0xFF;
	*/

	//memset(UUID, 0x00, MAX_LEN*sizeof(uint8_t));
	/*
    can_frame.data[0] = adc_sample_buffer1[0] & 0xFF;
    can_frame.data[1] = (adc_sample_buffer1[0] >> 8) & 0xFF;
    can_frame.data[2] = adc_sample_buffer2[0] & 0xFF;
    can_frame.data[3] = (adc_sample_buffer2[0] >> 8) & 0xFF;
    can_frame.data[4] = btnStatePlug & 0xFF;
    can_frame.data[5] = btnStateCharge & 0xFF;
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
	*/
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

void mfrc522_task_handler(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	while ( 1 )
    {
        status = MFRC522_Request(PICC_REQIDL, UUID);
		if ( status == MI_OK )
		{
			status = MFRC522_Anticoll(UUID);
			if ( status == MI_OK )
			{
				isRFIDCardDetected = true;
			}
			else
			{
				isRFIDCardDetected = false;
			}
		}
		else if ( status == MI_ERR )
		{
			//printf("RFID is not ready.\n");
			isRFIDCardDetected = false;
		}	

		//memset(UUID, 0x00, MAX_LEN*sizeof(uint8_t));
		k_sleep(K_MSEC(25));
    }
}

void encode_EVSE_voltAmp(uint8_t *canBuffer, struct EVSE_voltAmp *voltAmp)
{
	uint64_t canMsg = 0ULL;
	writeCanParam( &canMsg, 0, 16, voltAmp->presentVoltage, 0.1, 0 );
	writeCanParam( &canMsg, 16, 16, voltAmp->presentCurrent, 0.1, 0 );
	can64toCanBytes( canMsg, canBuffer );
}

void encode_EVSE_status(uint8_t *canBuffer, struct EVSE_status *status)
{
	uint64_t canMsg = 0ULL;
	writeCanBits( &canMsg, 0, 8, status->evseState );
	writeCanBits( &canMsg, 27, 3, status->CPstate );
	can64toCanBytes( canMsg, canBuffer );
}

void encode_EVSE_whTime(uint8_t *canBuffer, struct EVSE_whTime *whTime)
{
	uint64_t canMsg = 0ULL;
	writeCanBits( &canMsg, 0, 32, whTime->evseElapsedChargeTime );
	writeCanParam( &canMsg, 32, 32, whTime->evseDeliveredEnergy, 0.001, 0 );
	can64toCanBytes( canMsg, canBuffer );
}

void encode_SECC_MSG4(uint8_t *canBuffer, struct SECC_MSG4 *msg4)
{
	uint64_t canMsg = 0ULL;
	writeCanBits( &canMsg, 16, 8, msg4->EVStatus_EVRESSSOC );
	can64toCanBytes( canMsg, canBuffer );
}

void writeCanBits(uint64_t *canMsg, int startBit, int bitLen, uint64_t bits)
{
    uint64_t mask;

    mask = (0xFFFFFFFFFFFFFFFF<<startBit) & (0xFFFFFFFFFFFFFFFF>>(64-(startBit+bitLen)));
    bits<<=startBit;

    *canMsg &=~mask;
    *canMsg |= (mask&bits);
}

void writeCanParam(uint64_t *canMsg, int startBit, int bitLen, float param, float resl, float ofset)
{
    uint64_t mask;
    uint64_t bits=(param-ofset)/resl;
    mask = (0xFFFFFFFFFFFFFFFF<<startBit) & (0xFFFFFFFFFFFFFFFF>>(64-(startBit+bitLen)));
    bits<<=startBit;
    *canMsg &=~mask;
    *canMsg |= (mask&bits);
}

void can64toCanBytes(uint64_t can64, uint8_t *canBytes)
{
    int i;
    uint8_t *ptr=(uint8_t *)&can64;
    for(i=0; i<8; i++)
	{
        canBytes[i]= ptr[7-i];
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	long res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if( res < out_min )
	{
		return out_min;
	} 
	else if ( res > out_max ) 
	{
		return out_max;
	} 
	else
	{
		return res;
	}
}