/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file led_states.hpp
 *
 * Defines uORB over UAVCANv1 led_states subscriber
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <uORB/topics/led_states.h>
#include <uORB/PublicationMulti.hpp>

#include "../DynamicPortSubscriber.hpp"

class UORB_over_UAVCAN_led_states_Subscriber : public UavcanDynamicPortSubscriber
{
public:
	UORB_over_UAVCAN_led_states_Subscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "led_states", instance) { };

	void subscribe() override
	{
		// Subscribe to messages uORB led_states payload over UAVCAN
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  sizeof(struct led_states_s),
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC * 10000,
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{
		PX4_INFO("uORB led_states Callback");

		if (receive.payload_size == sizeof(struct led_states_s)) {
			led_states_s *led_states_msg = (led_states_s *)receive.payload;
			led_states_msg->timestamp = hrt_absolute_time();

			/* As long as we don't have timesync between nodes we set the timestamp to the current time */

			_led_states_pub.publish(*led_states_msg);

		} else {
			PX4_ERR("uORB over UAVCAN %s playload size mismatch got %d expected %d",
				_subj_sub._subject_name, receive.payload_size, sizeof(struct led_states_s));
		}
	};

private:
	uORB::PublicationMulti<led_states_s> _led_states_pub{ORB_ID(led_states)};

};
