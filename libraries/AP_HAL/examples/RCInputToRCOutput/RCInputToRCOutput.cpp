/*
 RC input pass trough to RC output
 Max RC channels 14
 Max update rate 10 Hz
 Attention: If your board has safety switch,
 don't forget to push it to enable the PWM output.
 */
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>

#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Logger/AP_Logger.h>

//#include <AP_HAL_ChibiOS/hwdef/common/stdio.h>





void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;
static AP_SerialManager serial_manager;
AP_Int32 logger_bitmask;
static AP_Logger logger { logger_bitmask };

//cmd
#define RX_BUF_LEN 100
typedef struct uart_buf
{
	uint16_t rx_p,rx_p_last;
	uint8_t  rx_buf[RX_BUF_LEN];
}uart_buf_t;
uart_buf_t cmd;

uint16_t pwm_max = 1600, pwm_min = 1400, time_delay=2000;
bool on_off = false;

void setup(void)
{
	board_config.init();
	serial_manager.init();

	hal.rcout->set_freq(0xFF, 50);
	for (uint8_t i = 0; i < 8; i++)
	{
		hal.rcout->enable_ch(i);
	}
	hal.rcout->force_safety_off();
}

void new_sscanf(uint8_t *buf,int32_t *data)
{
	int32_t res = 0;
	for(uint8_t i=0;i<10;i++)
	{
		if(buf[i] >= '0' && buf[i] <= '9')
		{
			res = res*10 + buf[i] - 0x30;
		}else
		{
			break;
		}
	}
	*data = res;
}

void decode_cmd(void)
{
	//usb decode rx
	cmd.rx_p = hal.console->available();
	if (cmd.rx_p == cmd.rx_p_last && cmd.rx_p > 0)
	{
		for (uint16_t i = 0; i < cmd.rx_p; i++)
		{
			cmd.rx_buf[i] = hal.console->read();
		}
		//decode pwm max
		if (cmd.rx_buf[0] == 'p' && cmd.rx_buf[1] == 'w' && cmd.rx_buf[2] == 'm' && cmd.rx_buf[3] == '_' &&
			cmd.rx_buf[4] == 'm' && cmd.rx_buf[5] == 'a' && cmd.rx_buf[6] == 'x' && cmd.rx_buf[7] == '=')
		{
			int32_t data;
			new_sscanf(&cmd.rx_buf[8], &data);
			hal.console->printf("pwm_max:%d \n",data);
			if (data > 800 && data < 2200)
			{
				pwm_max = data;
			}
		}

		//decode pwm min
		if (cmd.rx_buf[0] == 'p' && cmd.rx_buf[1] == 'w' && cmd.rx_buf[2] == 'm' && cmd.rx_buf[3] == '_' &&
			cmd.rx_buf[4] == 'm' && cmd.rx_buf[5] == 'i' && cmd.rx_buf[6] == 'n' && cmd.rx_buf[7] == '=')
		{
			int32_t data;
			new_sscanf(&cmd.rx_buf[8], &data);
			hal.console->printf("pwm_min:%d \n",data);
			if (data > 800 && data < 2200)
			{
				pwm_min = data;
			}
		}

		//decode time
		if (cmd.rx_buf[0] == 't' && cmd.rx_buf[1] == 'i' && cmd.rx_buf[2] == 'm' && cmd.rx_buf[3] == 'e' && cmd.rx_buf[4] == '=')
		{
			int32_t data;
			new_sscanf(&cmd.rx_buf[5], &data);
			hal.console->printf("time:%d \n",data);
			if (data > 0 && data < 1000 * 10)
			{
				time_delay = data;
			}
		}

		//decode start stop
		//decode time
		if (cmd.rx_buf[0] == 's' && cmd.rx_buf[1] == 't' && cmd.rx_buf[2] == 'a' && cmd.rx_buf[3] == 'r' && cmd.rx_buf[4] == 't')
		{
			on_off = true;
		}else if (cmd.rx_buf[0] == 's' && cmd.rx_buf[1] == 't' && cmd.rx_buf[2] == 'o' && cmd.rx_buf[3] == 'p')
		{
			on_off =false;
		}
		memset(cmd.rx_buf,0,cmd.rx_p);
	}
	else
	{
		cmd.rx_p_last = cmd.rx_p;
	}
}

void loop(void)
{
	static uint32_t last_ms1 = 0,last_ms2 = 0;
	static bool pwm_switch = true;
	uint32_t ms = AP_HAL::millis();

	if ((ms - last_ms2) > time_delay && on_off == true)
	{
		last_ms2 = ms;
		if (pwm_switch == false)
		{
			pwm_switch = true;
			//pwm_min
			hal.rcout->write(0, pwm_min);
		}else
		{
			pwm_switch = false;
			//pwm_max
			hal.rcout->write(0, pwm_max);
		}
	}

	decode_cmd();
	if ((ms - last_ms1) > 1000)
	{
		last_ms1 = ms;
		hal.console->printf("pwm max:%d pwm min:%d time:%d sys_time:%d \n",pwm_max,pwm_min,time_delay,ms/1000);
	}
	hal.scheduler->delay(1);
}
const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] =
{
	AP_GROUPEND
};
GCS_Dummy _gcs;
AP_HAL_MAIN();
