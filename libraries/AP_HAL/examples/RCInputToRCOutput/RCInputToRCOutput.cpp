/*
 RC input pass trough to RC output
 Max RC channels 14
 Max update rate 10 Hz
 Attention: If your board has safety switch,
 don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;
static AP_InertialSensor ins;

static Compass compass;

static AP_GPS gps;
static AP_Baro barometer;
static AP_SerialManager serial_manager;
AP_Int32 logger_bitmask;
static AP_Logger logger
{ logger_bitmask };

#define MAX_CHANNELS_RX 10
static uint16_t value_rx[MAX_CHANNELS_RX];

#define MAX_CHANNELS_TX 10

#define RX_BUF_LEN2 100
uint8_t rxbuf2[RX_BUF_LEN2] =
{ 0 };
uint16_t rx_p2 = 0;

#define RX_BUF_LEN1 100
uint8_t rxbuf1[RX_BUF_LEN1] =
{ 0 };
uint16_t rx_p1 = 0;

void setup(void)
{
	board_config.init();
	serial_manager.init();

	hal.rcout->set_freq(0xFF, 490);
	for (uint8_t i = 0; i < MAX_CHANNELS_TX; i++)
	{
		hal.rcout->enable_ch(i);
	}
	hal.rcout->force_safety_off();
}

void loop(void)
{
	static uint32_t last_ms = 0;
	uint32_t ms = AP_HAL::millis();

	//rx
	uint8_t rx_channels = hal.rcin->num_channels(); // Get the numbers channels detected by RC_INPUT.
	if (rx_channels > MAX_CHANNELS_RX)
	{
		rx_channels = MAX_CHANNELS_RX;
	}
	for (uint8_t i = 0; i < rx_channels; i++)
	{
		value_rx[i] = hal.rcin->read(i);
	}

	//tx
	hal.rcout->set_freq(0xFF, 50);
	for (uint8_t i = 0; i < MAX_CHANNELS_TX; i++)
	{
		hal.rcout->write(i, value_rx[i]);
	}

	//telem2
	uint16_t rxnum2 = hal.uartD->available();
	if (rxnum2 > RX_BUF_LEN2)
	{
		rxnum2 = RX_BUF_LEN2;
	}
	for (uint16_t i = 0; i < rxnum2; i++)
	{
		rxbuf2[i] = hal.uartD->read();
		rx_p2 = i;
	}
	if (rx_p2 > 0)
	{
		hal.uartD->write(rxbuf2, rx_p2);
		rx_p2 = 0;
	}

	//telem1
	uint16_t rxnum1 = hal.uartC->available();
	if (rxnum1 > RX_BUF_LEN1)
	{
		rxnum1 = RX_BUF_LEN1;
	}
	for (uint16_t i = 0; i < rxnum1; i++)
	{
		rxbuf1[i] = hal.uartC->read();
		rx_p1 = i;
	}
	if (rx_p1 > 0)
	{
		hal.uartD->write(rxbuf1, rx_p1);
		rx_p1 = 0;
	}

	if ((ms - last_ms) > 250)
	{
		last_ms = ms;
		printf("rx_c: %2u ", rx_channels);
		for (uint8_t i = 0; i < rx_channels; i++)
		{
			printf("%2u:%04u ", (unsigned) i + 1, (unsigned) value_rx[i]);
		}
		printf("Time:%d ", ms);
		printf("\n");

//	    hal.uartD->printf("232 telem2 \n");
//	    hal.uartC->printf("232 telem1 \n");
	}
	hal.scheduler->delay(5);
}
const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] =
{
AP_GROUPEND };
GCS_Dummy _gcs;
AP_HAL_MAIN();
