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

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/
#include "GCS.h"
#include "GCS_MAVLink.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#ifdef MAVLINK_SEPARATE_HELPERS
// Shut up warnings about missing declarations; TODO: should be fixed on
// mavlink/pymavlink project for when MAVLINK_SEPARATE_HELPERS is defined
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "include/mavlink/v2.0/mavlink_helpers.h"
#pragma GCC diagnostic pop
#endif

AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

// per-channel lock
static HAL_Semaphore chan_locks[MAVLINK_COMM_NUM_BUFFERS];

mavlink_system_t mavlink_system = {7,1};

// mask of serial ports disabled to allow for SERIAL_CONTROL
static uint8_t mavlink_locked_mask;

// routing table
MAVLink_routing GCS_MAVLINK::routing;

/*
  lock a channel, preventing use by MAVLink
 */
void GCS_MAVLINK::lock_channel(mavlink_channel_t _chan, bool lock)
{
    if (!valid_channel(chan)) {
        return;
    }
    if (lock) {
        mavlink_locked_mask |= (1U<<(unsigned)_chan);
    } else {
        mavlink_locked_mask &= ~(1U<<(unsigned)_chan);
    }
}

bool GCS_MAVLINK::locked() const
{
    return (1U<<chan) & mavlink_locked_mask;
}

// set a channel as private. Private channels get sent heartbeats, but
// don't get broadcast packets or forwarded packets
void GCS_MAVLINK::set_channel_private(mavlink_channel_t _chan)
{
    const uint8_t mask = (1U<<(unsigned)_chan);
    mavlink_private |= mask;
    mavlink_active &= ~mask;
}

// return a MAVLink parameter type given a AP_Param type
MAV_PARAM_TYPE GCS_MAVLINK::mav_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
	    return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
	    return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    if (!valid_channel(chan)) {
        return 0;
    }
    if ((1U<<chan) & mavlink_locked_mask) {
        return 0;
    }
	int16_t ret = mavlink_comm_port[chan]->txspace();
	if (ret < 0) {
		ret = 0;
	}
    return (uint16_t)ret;
}

/*
  send a buffer out a MAVLink channel
 */
uint8_t new_buf[256 + 18]={0};
uint8_t pack_len = 0,init=0,head_flag=0;
uint64_t mac = 0x13A2004127476B;
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (written < len) {
        AP_HAL::panic("Short write on UART: %lu < %u", (unsigned long)written, len);
    }
#else
    if(init==0)
	{
		init=1;
		new_buf[0] = 0x7E;
		new_buf[1] = (14 + len) & 0xFF00;
		new_buf[2] = (14 + len) & 0x00FF;
		new_buf[3] = 0x10;
		new_buf[4] = 0x00;

		new_buf[5] = mac >> 56 & 0xFF;
		new_buf[6] = mac >> 48 & 0xFF;
		new_buf[7] = mac >> 40 & 0xFF;
		new_buf[8] = mac >> 32 & 0xFF;
		new_buf[9] = mac >> 24 & 0xFF;
		new_buf[10] = mac >> 16 & 0xFF;
		new_buf[11] = mac >> 8 & 0xFF;
		new_buf[12] = mac >> 0 & 0xFF;

		new_buf[13] = 0xFF;
		new_buf[14] = 0xFE;

		new_buf[15] = 0;
		new_buf[16] = 0;
	}
	if(chan==0)
	{
		mavlink_comm_port[chan]->write(buf, len);
	}else if (len == 6 && buf[0] == 0xFE)
	{
		memcpy(new_buf + 17, buf, 6);
		head_flag=1;
	}
	else if (head_flag==1)
	{
		memcpy(new_buf + 17 + 6, buf, len);
		pack_len = len;
		head_flag=0;
	}
	else if (len == 2 && head_flag==0)
	{
		memcpy(new_buf + 17 + 6 + pack_len, buf, 2);

		new_buf[1] = (14 + 6+pack_len+2) & 0xFF00;
		new_buf[2] = (14 + 6+pack_len+2) & 0x00FF;
		new_buf[17 + 6 + pack_len + 2] = 0;

		for (uint16_t i = 3; i < 17 + 6 + pack_len + 2; i++)
		{
			new_buf[17 + 6 + pack_len + 2] += new_buf[i];
		}
		new_buf[17 + 6 + pack_len + 2] = 0xFF - new_buf[17 + 6 + pack_len + 2];

		if (!valid_channel(chan))
		{
			return;
		}
		mavlink_comm_port[chan]->write(new_buf, 17 + 6 + pack_len + 2 + 1);
	}else
	{
		head_flag=0;
	}
#endif
}

/*
  lock a channel for send
 */
void comm_send_lock(mavlink_channel_t chan)
{
    chan_locks[(uint8_t)chan].take_blocking();
}

/*
  unlock a channel
 */
void comm_send_unlock(mavlink_channel_t chan)
{
    chan_locks[(uint8_t)chan].give();
}
