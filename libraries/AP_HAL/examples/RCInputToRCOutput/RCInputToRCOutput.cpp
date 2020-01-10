/*
 RC input pass trough to RC output
 Max RC channels 14
 Max update rate 10 Hz
 Attention: If your board has safety switch,
 don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>

#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Logger/AP_Logger.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;
static AP_SerialManager serial_manager;
AP_Int32 logger_bitmask;
static AP_Logger logger { logger_bitmask };



//SBUS
#define MAX_CHANNELS_RX 10
static uint16_t value_rx[MAX_CHANNELS_RX];

//PWM
#define MAX_CHANNELS_TX 10

//RS232
#define RX_BUF_LEN 100
typedef struct uart_buf
{
	uint16_t rx_p,rx_p_last;
	uint8_t  rx_buf[RX_BUF_LEN];
}uart_buf_t;
uart_buf_t rs232_1,rs232_2;

/*舵机到飞控(波特率:38400bps,数据每字节为10位,1位起始位,8位数据位,1位停止位,无奇偶校验,更新率:10ms,字节:18,一帧的传输时间大概:5.1ms)*/
/*
    MIN	MAX	MIN	    MAX
0	-30	30	3000	9000
1	-14	14	4600	7400
2	-35	35	2500	9500
3	-23	23	3700	8300
4	-30	30	3000	9000
5	-45	45	500	    9500
 */
struct PACKED rudToFcc
{
	unsigned char	uchrSynW1;			/*同步字1:0x55*/
	unsigned char	uchrSynW2;			/*同步字2:0xAA*/
	unsigned char	uchrAddress;		/*地址:0x23*/
	unsigned char	uchrCmdWord;		/*命令字*/
	signed   short	shrRudAgl1fdb;		/*舵偏角1反馈*/
	signed   short	shrRudAgl2fdb;		/*舵偏角2反馈*/
	signed   short	shrRudAgl3fdb;		/*舵偏角3反馈*/
	signed   short	shrRudAgl4fdb;		/*舵偏角4反馈*/
	signed   short	shrRudAgl5fdb;		/*舵偏角5反馈*/
	signed   short	shrRudAgl6fdb;		/*舵位移6反馈*/
	unsigned char	uchrCheckCode;		/*校验和:地址字节、命令字字节、舵偏角字节按字节异或*/
	unsigned char   uchrFrameEnd;		/*帧尾:0xF0*/
};
struct PACKED rudToFcc fccToRudMess,rx_control;

/*遥控数据通用帧结构*/
struct PACKED remoteControlGE
{
	unsigned char 	uchrSynNum1;		/*同步码1*/
	unsigned char	uchrSynNum2;		/*同步码2*/
	unsigned char	uchrPlaneType;		/*飞机类型*/
	unsigned char   uchrPlaneNum;		/*飞机编号*/
	unsigned char 	uchrSwitchCmd1;		/*遥控开关指令1*/
	unsigned char 	uchrSwitchCmd2;     /*遥控开关指令2*/
	unsigned char 	uchrSwitchCmd3;		/*遥控开关指令3*/
	unsigned char   uchrIdtCode;		/*标识码*/
	unsigned short	ushtCtrVal;			/*控制量*/
	unsigned char	ushrReserve1;		/*备用1*/
	signed	short	shrPitchAngle;		/*俯仰角操纵杆数据*/
	signed  short	shrRollAngle;		/*滚转角操纵杆数据*/
	unsigned short	ushrHrottletl;		/*油门操纵杆数据*/
	signed	short	shrPedalRudder;		/*脚蹬方向舵数据*/
	unsigned char	uchrPedalLBrake;	/*脚蹬左刹车数据*/
	unsigned char	uchrPedalRBrake;	/*脚蹬右刹车数据*/
	unsigned char	uchrHandSwStatus;	/*手摇开关状态*/
	signed   char	chrHandBalStatus;	/*手柄配平状态*/
	unsigned char	uchrReserve[11];/*备用*/
	unsigned char   uchrFrameCnt;       /*帧时计数器*/
	unsigned char   uchrCheckCode;      /*校验码*/
};
struct PACKED remoteControlGE remoteData;
unsigned char CheckByteXor(void *pMes,unsigned char len,unsigned char offset)
{
	unsigned short	i = 0;
	unsigned char	uchrSum = 0;
	unsigned char	*uchrPtr = NULL;

	uchrSum = (*((unsigned char*)pMes + offset));
	uchrPtr = ((unsigned char*)pMes + offset + 1);

	for(i = 0; i < (len -1); i++)
	{
		uchrSum ^= (*(uchrPtr + i));
	}

	return uchrSum;
}
unsigned char CheckAddSum(void *pMes,unsigned char len,unsigned char offset)
{
	unsigned short	i;
	unsigned int	uintSum = 0;
	unsigned char	*uchrPtr = NULL;

	uchrPtr = ((unsigned char*)pMes + offset);

	for(i = 0; i < len; i++)
	{
		uintSum += (*(uchrPtr + i));
	}

	return (uintSum & 0xFF);
}

signed short pwm_adc(uint16_t value, uint8_t dir)
{
	signed short data;
	if(dir > 0)
	{
		data = value - 1500;
		data *= 1;
		data = ((double)data/1000)*32767;
	}else
	{
		data = value - 1500;
		data *= -1;
		data = ((double)data/1000)*32767;
	}
	return data;
}

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
	static uint32_t last_ms1 = 0, last_ms2 = 0;
	uint32_t ms = AP_HAL::millis();

	//rx sbus
	uint8_t rx_channels = hal.rcin->num_channels(); // Get the numbers channels detected by RC_INPUT.
	if (rx_channels > MAX_CHANNELS_RX)
	{
		rx_channels = MAX_CHANNELS_RX;
	}
	for (uint8_t i = 0; i < rx_channels; i++)
	{
		value_rx[i] = hal.rcin->read(i);
	}

	//tx pwm
	hal.rcout->set_freq(0xFF, 50);
	for (uint8_t i = 0; i < MAX_CHANNELS_TX; i++)
	{
		hal.rcout->write(i, value_rx[i]);
	}

	//telem2 decode rx pwm
	rs232_2.rx_p = hal.uartD->available();
	if(rs232_2.rx_p == rs232_2.rx_p_last && rs232_2.rx_p > 0)
	{
		for (uint16_t i = 0; i < rs232_2.rx_p; i++)
		{
			rs232_2.rx_buf[i] = hal.uartD->read();
		}
		if (rs232_2.rx_p == sizeof(fccToRudMess))
		{
			memcpy(&fccToRudMess, rs232_2.rx_buf, sizeof(fccToRudMess));
		}
		if (fccToRudMess.uchrSynW1    == 0x55 &&
			fccToRudMess.uchrSynW2    == 0xAA &&
			fccToRudMess.uchrAddress  == 0x32 &&
			fccToRudMess.uchrCmdWord  == 0x42 &&
			fccToRudMess.uchrFrameEnd == 0xF0)
		{
			if (fccToRudMess.uchrCheckCode ==
				CheckByteXor(&fccToRudMess, (sizeof(fccToRudMess) - 4), 2) || 1)
			{
				memcpy(&rx_control, &fccToRudMess, sizeof(fccToRudMess));
			}
		}
	}else
	{
		rs232_2.rx_p_last = rs232_2.rx_p;
	}
	//telem1
//	rs232_1.rx_p = hal.uartC->available();
//	if (rs232_1.rx_p > RX_BUF_LEN)
//	{
//		rs232_1.rx_p = RX_BUF_LEN;
//	}
//	for (uint16_t i = 0; i < rs232_1.rx_p; i++)
//	{
//		rs232_1.rx_buf[i] = hal.uartC->read();
//	}
//	if (rs232_1.rx_p > 0)
//	{
//		hal.uartC->write(rs232_1.rx_buf, rs232_1.rx_p);
//		rs232_1.rx_p = 0;
//	}
	if ((ms - last_ms2) > 20)
	{
		last_ms2 = ms;
		remoteData.uchrSynNum1    = 0xEB;
		remoteData.uchrSynNum2    = 0x90;
		remoteData.uchrPlaneType  = 0x00;
		remoteData.uchrPlaneNum   = 0x00;
		remoteData.uchrSwitchCmd1 = 0x00;
		remoteData.uchrSwitchCmd2 = 0x00;
		remoteData.uchrSwitchCmd3 = 0x00;
		remoteData.uchrIdtCode    = 0x00;
		remoteData.ushtCtrVal     = 0x00;
		remoteData.ushrReserve1   = 0x00;
		remoteData.shrPitchAngle  = pwm_adc(value_rx[1],1);
		remoteData.shrRollAngle   = pwm_adc(value_rx[0],1);
		remoteData.ushrHrottletl  = pwm_adc(value_rx[2],1);
		remoteData.shrPedalRudder = pwm_adc(value_rx[3],1);
		remoteData.uchrPedalLBrake = 0;
		remoteData.uchrPedalRBrake = 1;
		remoteData.uchrHandSwStatus = 0x03;
		remoteData.chrHandBalStatus = 0x00;
		memset(remoteData.uchrReserve,0,11);
		remoteData.uchrFrameCnt++;
		remoteData.uchrCheckCode = CheckAddSum(&remoteData,sizeof(remoteData) - 1,0);
		hal.uartC->write((uint8_t *)&remoteData,sizeof(remoteData));
	}

	if ((ms - last_ms1) > 250)
	{
		last_ms1 = ms;
		printf("rx_c: %2u ", rx_channels);
		for (uint8_t i = 0; i < rx_channels; i++)
		{
			printf("%2u:%04u ", (unsigned) i + 1, (unsigned) value_rx[i]);
		}
		printf("Time:%d ", ms);
		printf("\n");

	    hal.uartD->printf("1:%d 2:%d 3:%d 4:%d 5:%d 6:%d \n",
	    		fccToRudMess.shrRudAgl1fdb,
				fccToRudMess.shrRudAgl2fdb,
				fccToRudMess.shrRudAgl3fdb,
				fccToRudMess.shrRudAgl4fdb,
				fccToRudMess.shrRudAgl5fdb,
				fccToRudMess.shrRudAgl6fdb);
	}
	hal.scheduler->delay(1);
}
const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] =
{
	AP_GROUPEND
};
GCS_Dummy _gcs;
AP_HAL_MAIN();
