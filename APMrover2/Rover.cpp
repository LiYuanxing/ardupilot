/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Randy Mackay, Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See http://dev.ardupilot.org for details
*/

#include "Rover.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#include "AP_Gripper/AP_Gripper.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(Rover, &rover, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    SCHED_TASK(read_radio,             50,    200),
    SCHED_TASK(ahrs_update,           400,    400),
    SCHED_TASK(read_rangefinders,      50,    200),
    SCHED_TASK(update_current_mode,   400,    200),
    SCHED_TASK(set_servos,            400,    200),
    SCHED_TASK(update_GPS,             50,    300),
    SCHED_TASK_CLASS(AP_Baro,             &rover.barometer,        update,         10,  200),
    SCHED_TASK_CLASS(AP_Beacon,           &rover.g2.beacon,        update,         50,  200),
    SCHED_TASK_CLASS(AP_Proximity,        &rover.g2.proximity,     update,         50,  200),
    SCHED_TASK_CLASS(AP_WindVane,         &rover.g2.windvane,      update,         20,  100),
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_VisualOdom,       &rover.g2.visual_odom,   update,         50,  200),
#endif
    SCHED_TASK_CLASS(AC_Fence,            &rover.g2.fence,         update,         10,  100),
    SCHED_TASK(update_wheel_encoder,   50,    200),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(update_mission,         50,    200),
    SCHED_TASK(update_logging1,        10,    200),
    SCHED_TASK(update_logging2,        10,    200),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_receive,                    400,    500),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&rover._gcs,       update_send,                       400,   1000),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_mode_switch,        7,    200),
    SCHED_TASK_CLASS(RC_Channels,         (RC_Channels*)&rover.g2.rc_channels, read_aux_all,           10,    200),
    SCHED_TASK_CLASS(AP_BattMonitor,      &rover.battery,          read,           10,  300),
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &rover.ServoRelayEvents, update_events,  50,  200),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &rover.g2.gripper,      update,         10,   75),
#endif
    SCHED_TASK(rpm_update,             10,    100),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &rover.camera_mount,     update,         50,  200),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &rover.camera,           update_trigger, 50,  200),
#endif
    SCHED_TASK(gcs_failsafe_check,     10,    200),
    SCHED_TASK(fence_check,            10,    200),
    SCHED_TASK(ekf_check,              10,    100),
    SCHED_TASK_CLASS(ModeSmartRTL,        &rover.mode_smartrtl,    save_position,   3,  200),
    SCHED_TASK_CLASS(AP_Notify,           &rover.notify,           update,         50,  300),
    SCHED_TASK(one_second_loop,         1,   1500),
    SCHED_TASK_CLASS(AC_Sprayer,          &rover.g2.sprayer,           update,      3,  90),
    SCHED_TASK_CLASS(Compass,          &rover.compass,              cal_update, 50, 200),
    SCHED_TASK(compass_save,           0.1,   200),
    SCHED_TASK(accel_cal_update,       10,    200),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger,     &rover.logger,        periodic_tasks, 50,  300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &rover.ins,              periodic,      400,  200),
    SCHED_TASK_CLASS(AP_Scheduler,        &rover.scheduler,        update_logging, 0.1, 200),
    SCHED_TASK_CLASS(AP_Button,           &rover.button,           update,          5,  200),
#if STATS_ENABLED == ENABLED
    SCHED_TASK(stats_update,            1,    200),
#endif
    SCHED_TASK(crash_check,            10,    200),
    SCHED_TASK(cruise_learn_update,    50,    200),
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,           10,    200),
#endif
    SCHED_TASK(read_airspeed,          10,    100),
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info,        1,     10),
#endif
#ifdef USER_COUDE
    SCHED_TASK(user_code,             50,    200),
	SCHED_TASK(user_guided,           1,    200),
#endif
};

constexpr int8_t Rover::_failsafe_priorities[7];

Rover::Rover(void) :
    AP_Vehicle(),
    param_loader(var_info),
    channel_steer(nullptr),
    channel_throttle(nullptr),
    channel_lateral(nullptr),
	channel_return(nullptr),
    logger{g.log_bitmask},
    modes(&g.mode1),
    control_mode(&mode_initializing),
    G_Dt(0.02f)
{
}

#if STATS_ENABLED == ENABLED
/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
    g2.stats.set_flying(g2.motors.active());
    g2.stats.update();
}
#endif

/*
  setup is called when the sketch starts
 */
void Rover::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}

// update AHRS system
void Rover::ahrs_update()
{
    arming.update_soft_armed();

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs().update();
#endif

    // AHRS may use movement to calculate heading
    update_ahrs_flyforward();

    ahrs.update();

    // update position
    have_position = ahrs.get_position(current_loc);

    // set home from EKF if necessary and possible
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = norm(velocity.x, velocity.y);
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
    if (!g.fs_gcs_enabled) {
        // gcs failsafe disabled
        return;
    }

    // check for updates from GCS within 2 seconds
    failsafe_trigger(FAILSAFE_EVENT_GCS, failsafe.last_heartbeat_ms != 0 && (millis() - failsafe.last_heartbeat_ms) > 2000);
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Sail();
    }

    if (should_log(MASK_LOG_THR)) {
        Log_Write_Throttle();
        logger.Write_Beacon(g2.beacon);
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
    }

    if (should_log(MASK_LOG_RANGEFINDER)) {
        logger.Write_Proximity(g2.proximity);
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        Log_Write_Steering();
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
        g2.wheel_encoder.Log_Write();
    }

    if (should_log(MASK_LOG_IMU)) {
        logger.Write_Vibration();
    }
}


/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    // allow orientation change at runtime to aid config
    ahrs.update_orientation();

    set_control_channels();

    // cope with changes to aux functions
    SRV_Channels::enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;
    AP_Notify::flags.flying = hal.util->get_soft_armed();

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // attempt to update home position and baro calibration if not armed:
    if (!hal.util->get_soft_armed()) {
        update_home();
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());

    // send latest param values to wp_nav
    g2.wp_nav.set_turn_params(g.turn_max_g, g2.turn_radius, g2.motors.have_skid_steering());
}

void Rover::update_GPS(void)
{
    gps.update();
    if (gps.last_message_time_ms() != last_gps_msg_ms) {
        last_gps_msg_ms = gps.last_message_time_ms();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Rover::update_current_mode(void)
{
    // check for emergency stop
    if (SRV_Channels::get_emergency_stop()) {
        // relax controllers, motor stopping done at output level
        g2.attitude_control.relax_I();
    }

    control_mode->update();
}

// update mission including starting or stopping commands. called by scheduler at 10Hz
void Rover::update_mission(void)
{
    if (control_mode == &mode_auto) {
        if (ahrs.home_is_set() && mode_auto.mission.num_commands() > 1) {
            mode_auto.mission.update();
        }
    }
}
#ifdef USER_COUDE
struct PACKED bwbot {
	uint8_t  head1;				//0xcd
	uint8_t  head2;				//0xeb
	uint8_t  head3;				//0xd7
	uint8_t  len;				//0x37
	float    power_charger; 	//当前充电极片电压，单位 V。
	uint8_t  sum1;
	float    power_battery; 	//当前电池电压，单位 V。
	uint8_t  sum2;
	float    current; 			//当前充电电流，单位 A。
	uint8_t  sum3;
	uint32_t left_sensor1; //左侧第一个红外传感器探测到的信号值，参考下文定义。
	uint8_t  sum4;
	uint32_t left_sensor2; //左侧第二个红外传感器探测到的信号值，参考下文定义。
	uint8_t  sum5;
	uint32_t right_sensor1; //右侧第一个红外传感器探测到的信号值，参考下文定义。
	uint8_t  sum6;
	uint32_t right_sensor2; //右侧第二个红外传感器探测到的信号值，参考下文定义。
	uint8_t  sum7;
	float distance1; //超声波模块测距值，单位 mm。
	uint8_t  sum8;
	float distance2; //保留，扩展用。
	uint8_t  sum9;
	uint32_t time_stamp; //时间戳,单位为 2 毫秒， 用于统计丢包率。
	uint8_t  sum10;
	uint32_t version; //版本号， 当前值为 3。
	uint8_t  sum11;
};


bool Rover::check_bwbot(uint8_t *p,uint16_t len)
{
	if(len != 59)
	{
		return false;
	}

	if(p[0] != 0xcd || p[1] != 0xeb || p[2] != 0xd7 || p[3] != 0x37)
	{
		return false;
	}

	for(uint8_t i=0;i<11;i++)
	{
		uint8_t check = p[4+i*5] +p[5+i*5] +p[6+i*5] +p[7+i*5];
		if(check != p[8+i*5])
		{
			return false;
		}
	}
	return true;
}
#define BUF_LEN 120
float   h_steering, h_throttle;
#define MOVE_THR 25

double a_lat,a_lng;
float  t_yaw,a_yaw;
uint8_t a_target_dir;
uint8_t rover_reached_destination;
uint8_t a_up,a_down,a_left,a_right;

void Rover::user_code(void)
{
#if 1
	static uint32_t time_p = 0 , error_time = 0;
	bool   check_res = false;
	uint8_t buf[BUF_LEN];
	struct PACKED bwbot data;
	uint16_t p = 0;
	static uint8_t step_num = 0;
	static uint8_t fil_count = 0;
	uint8_t  id= g.sysid_this_mav;
	static uint8_t init_charge = 0;
#endif

	AP_HAL::UARTDriver *user_uart1;
	user_uart1 = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HongWai, 0);
	if (user_uart1 != nullptr)
	{
		user_uart1->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HongWai, 0));
		uint16_t now_num = user_uart1->available();
		if (now_num >= 59)
		{
			uint16_t j = 0;
			for (uint16_t i = 0; i < now_num; i++)
			{
				j = (j >= BUF_LEN) ? 0 : j;
				buf[j] = user_uart1->read();
				j++;
			}
			for (uint16_t i = 0; i < 70; i++)
			{
				if (buf[i] == 0xcd && buf[i + 1] == 0xeb && buf[i + 2] == 0xd7 && buf[i + 3] == 0x37)
				{
					p = i;
					check_res = check_bwbot(buf + i, 59);
					if (check_res)
					{
						break;
					}
				}
			}
		}

		if (check_res == true && id > 1 )
		{
			memcpy((uint8_t*) &data, buf + p, sizeof(data));
			/*************************init****************************/
			if(control_mode == &mode_manual &&
			   channel_return->get_control_in() > 0 &&
			   channel_return->get_control_in() < 1700 &&
			   init_charge ==0 )//test charge
			{
				init_charge = 1;
				step_num = 2;
				printf("test charge\n");
				error_time = AP_HAL::millis();
			}else if(rover_reached_destination == 1 && a_target_dir > 0 )
			{
				rover_reached_destination = 0;
				a_target_dir = 0;
				set_mode(0, ModeReason::UNKNOWN);//mode_manual
				step_num = 0;
				init_charge = 1;
				printf("guided ok\n");
				error_time = AP_HAL::millis();
			}

			/*************************break**************************/
			if(control_mode != &mode_manual)
			{
				//init
				init_charge = 0;
				step_num = 0;
				h_steering = 0;
				h_throttle = 0;
				error_time = AP_HAL::millis();
			}

			/*************************run****************************/
			if(init_charge == 1)
			{
				if(data.distance1/10 < 15)//stop
				{
					h_steering = h_throttle = 0;
					arming.disarm();
				}
				float yaw = 0;
				switch (step_num)
				{
				case 0:
					if ((AP_HAL::millis() - error_time) > 2 * 1000)
					{
						error_time = AP_HAL::millis(); //update time
						step_num++; //next statu
						h_steering = 0;
						h_throttle = 1;
						rover_reached_destination = 0;
					}
					break;
				case 1:
					if ((AP_HAL::millis() - error_time) > 10 * 1000)
					{
						h_steering = h_throttle = 0; //error wait
						fil_count = 0;
					}
					h_throttle = 0;
					h_steering = 700;
					yaw = ahrs.yaw*RAD_TO_DEG - t_yaw;
					yaw = (yaw > 360)?(yaw-360):yaw;
					yaw = (yaw < 0  )?(yaw+360):yaw;
					if(fabs(yaw)<10)
					{
						error_time = AP_HAL::millis(); //update time
						step_num++; //next statue

						h_steering = 0;
						rover_reached_destination = 0;
					}
					break;
				case 2: //move up to find l1 l2
					h_throttle = MOVE_THR; //0-100
					if ((AP_HAL::millis() - error_time) > 10 * 1000)
					{
						h_steering = h_throttle = 0; //error wait
						fil_count = 0;
					}
					else if (data.left_sensor1 > 0)
					{
						h_throttle = 0; //stop move
						h_steering = 700; // turn right
						error_time = AP_HAL::millis(); //update time
						fil_count++;
						if(fil_count > 5)
						{
							fil_count = 0;
							step_num++; //next statue
						}
					}
					else if (data.right_sensor2 > 0)
					{
						h_throttle = 0; //stop move
						h_steering = -700; //turn left
						error_time = AP_HAL::millis(); //update time
						fil_count++;
						if(fil_count > 5)
						{
							fil_count = 0;
							step_num++; //next statue
						}
					}

					break;
				case 3://wait turn round
					if ((AP_HAL::millis() - error_time) > 10 * 1000)
					{
						h_steering = h_throttle = 0; //error wait
						fil_count = 0;
					}
					else if (data.left_sensor2 > 0 && data.right_sensor1 > 0)
					{
						h_steering = 0;
						h_throttle = -MOVE_THR+10;
						error_time = AP_HAL::millis(); //update time
						fil_count++;
						if(fil_count > 2)
						{
							fil_count = 0;
							step_num++; //next statue
						}
					}
					break;
				case 4://wait distance
					if (data.distance1 / 10 < 20)
					{
						h_steering = 0;
						h_throttle = 0;
						error_time = AP_HAL::millis(); //update time
						step_num++; //next statue
					}else if ((AP_HAL::millis() - error_time) > 20 * 1000)
					{
						h_steering = h_throttle = 0; //error wait
						fil_count = 0;
					}else if(((data.left_sensor1 ==2 || data.left_sensor2 ==2) && data.right_sensor1 == 0 && data.right_sensor2 == 0) ||
							(data.left_sensor2 == 2 && data.right_sensor1 == 2))
					{
						h_steering = 150;
						fil_count = 0;
					}else if(((data.right_sensor1 ==1 || data.right_sensor2 ==1) && data.left_sensor1 == 0 && data.left_sensor2 == 0) ||
							(data.left_sensor2 == 1 && data.right_sensor1 == 1))
					{
						h_steering = -150;
						fil_count = 0;
					}else if(data.left_sensor2 ==3 && data.right_sensor1 <3)
					{
						h_steering = 150;
						fil_count = 0;
					}else if(data.left_sensor2 <3 && data.right_sensor1 == 3)
					{
						h_steering = -150;
						fil_count = 0;
					}else if(data.left_sensor2 ==3 && data.right_sensor1 == 3)
					{
						h_steering = 0;
					}
					break;
				case 5:
					error_time = AP_HAL::millis(); //ok
					break;
				default:
					step_num = 0;
					h_steering = 0;
					h_throttle = 0;
					break;
				}
			}

			if ((AP_HAL::millis() - time_p) > 1000)
			{
				time_p = AP_HAL::millis();
#if 1
//				printf("step:%d   on:%d  %d   %d   %d   %d   %3.2f   \n",
//						step_num,channel_return->get_control_in(),
//						data.left_sensor1,data.left_sensor2,
//						data.right_sensor1,data.right_sensor2,
//						data.distance1/10);
//				printf("vol:%f \n",battery.voltage());
//				printf("control_mode:%d get_control_in:%d init_charge:%d",control_mode,channel_return->get_control_in(),init_charge);
				printf("init_charge:%d step_num:%d h_steering:%.1f h_throttle:%.1f \n",
						init_charge,step_num,h_steering,h_throttle);
#endif
			}
		}
	}
#if 0
	AP_HAL::UARTDriver *user_uart1;
	AP_HAL::UARTDriver *user_uart2;
	user_uart1 = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HongWai, 0);
	user_uart2 = serial_manager.find_serial(AP_SerialManager::SerialProtocol_HongWai, 1);
	if (user_uart1 != nullptr)
	{
		user_uart1->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HongWai, 0));
		uint16_t now_num = user_uart1->available();
		for (uint16_t i = 0; i < now_num; i++)
		{
			printf("%02x ",user_uart1->read());
//			printf("%d\r\n",serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HongWai,0));
		}
	}
//	else
//	{
//		printf("n1\r\n");
//	}
	if (user_uart2 != nullptr)
	{
		user_uart2->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HongWai, 2));
		uint16_t now_num = user_uart2->available();
		for (uint16_t i = 0; i < now_num; i++)
		{
			printf("%02x ",user_uart2->read());
//			printf("%d\r\n",serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_HongWai,1));
		}
	}
//	else
//	{
//		printf("n1\r\n");
//	}
#else
#endif
}

uint8_t my_sys_id = 0;
void Rover::user_guided(void)
{
#if 1
	uint8_t  id= g.sysid_this_mav;
	my_sys_id = id;
	char key[AP_MAX_NAME_SIZE+1]={0};
	AP_Param *vp;
	enum ap_var_type var_type = AP_PARAM_FLOAT;
	memset(key, 0, AP_MAX_NAME_SIZE + 1);
	strcpy(key, "BATT_ARM_VOLT");
	vp = AP_Param::find(key, &var_type);
	if (vp != nullptr && my_sys_id != 1)
	{
		if(battery.voltage() < vp->cast_to_float(var_type))
		{
			gcs().request_charge_send(id,1);
			gcs().send_text(MAV_SEVERITY_INFO,("num:%d send req\n"),id);
		}
	}
	if(channel_return->get_control_in() > 1700 && my_sys_id != 1)
	{
		gcs().request_charge_send(id,1);
		gcs().send_text(MAV_SEVERITY_INFO,("num:%d send req\n"),id);
	}
#endif
}
#endif

#if OSD_ENABLED == ENABLED
void Rover::publish_osd_info()
{
    AP_OSD::NavInfo nav_info {0};
    if (control_mode == &mode_loiter) {
        nav_info.wp_xtrack_error = control_mode->get_distance_to_destination();
    } else {
        nav_info.wp_xtrack_error = control_mode->crosstrack_error();
    }
    nav_info.wp_distance = control_mode->get_distance_to_destination();
    nav_info.wp_bearing = control_mode->wp_bearing() * 100.0f;
    if (control_mode == &mode_auto) {
         nav_info.wp_number = mode_auto.mission.get_current_nav_index();
    }
    osd.set_nav_info(nav_info);
}
#endif

Rover rover;

AP_HAL_MAIN_CALLBACKS(&rover);
