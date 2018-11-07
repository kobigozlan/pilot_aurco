#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */
Quaternion copterRotInv;
Vector3f lidarDirection, optFeature, oldOptFeature, relativeOptFeature;
float roll_out, pitch_out, roll_err, pitch_err,temp,ref_alt,p_flow,d_flow,i_flow,i_flow_max;
uint32_t lastTime = 0;
bool first_init = true;
//KF2D kf_2d_opicalflow;
// althold_init - initialise althold controller
bool Copter::ModeAltHold::init(bool ignore_checks) {
	// initialize vertical speeds and leash lengths
	pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
	pos_control->set_accel_z(g.pilot_accel_z);

	// initialise position and desired velocity
	if (!pos_control->is_active_z()) {
		pos_control->set_alt_target_to_current_alt();
		pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
	}

	// stop takeoff if running
	takeoff_stop();

	//reset data
	first_init = true;
	optFeature.x = 0;
	optFeature.y = 0;
	optFeature.z = 0;
	oldOptFeature.x = 0;
	oldOptFeature.y = 0;
	oldOptFeature.z = 0;
	roll_out =0;
	pitch_out=0;
	roll_err = 0;
	pitch_err = 0;
	//KF
	//kf_2d_opicalflow = KF2D();
	ref_alt = ahrs.ref_alt.cast_to_float();
	p_flow = ahrs.p_flow.cast_to_float();
	d_flow = ahrs.d_flow.cast_to_float();
	i_flow = ahrs.i_flow.cast_to_float();
	i_flow_max = ahrs.i_flow_max.cast_to_float()*1000;
//	hal.console->printf("init : (p : %.2f,d : %.2f,i : %.2f,ref : %.2f)\r\n", p_flow,d_flow,i_flow,ref_alt);
	return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more

// get pilot desired lean angles


void Copter::ModeAltHold::run() {
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;

	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
	pos_control->set_accel_z(g.pilot_accel_z);

	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(channel_roll->get_control_in(),
			channel_pitch->get_control_in(), target_roll, target_pitch,
			attitude_control->get_althold_lean_angle_max());
	//get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), roll_out, pitch_out, attitude_control->get_althold_lean_angle_max());
	//hal.console->printf("R:%f ,P:%f\r\n",target_roll, target_pitch);
	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(
			channel_yaw->get_control_in());

	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(
			channel_throttle->get_control_in());
	target_climb_rate = constrain_float(target_climb_rate,
			-get_pilot_speed_dn(), g.pilot_speed_up);

	// Alt Hold State Machine Determination
	if (!motors->armed() || !motors->get_interlock()) {
		althold_state = AltHold_MotorStopped;
	} else if (takeoff_state.running || takeoff_triggered(target_climb_rate)) {
		althold_state = AltHold_Takeoff;
	} else if (!ap.auto_armed || ap.land_complete) {
		althold_state = AltHold_Landed;
	} else {
		althold_state = AltHold_Flying;
	}

	// Alt Hold State Machine
	switch (althold_state) {

	case AltHold_MotorStopped:

		motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());
		attitude_control->reset_rate_controller_I_terms();
		attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
		// force descent rate and call position controller
		pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
		heli_flags.init_targets_on_arming=true;
#else
		pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
#endif
		pos_control->update_z_controller();
		break;

	case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
		if (heli_flags.init_targets_on_arming) {
			heli_flags.init_targets_on_arming=false;
		}
#endif
		// set motors to full range
		motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

		// initiate take-off
		if (!takeoff_state.running) {
			takeoff_timer_start(
					constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
		}

		// get take-off adjusted pilot and takeoff climb rates
		takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

		// get avoidance adjusted climb rate
		target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

		// call attitude controller
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt,
				false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
		pos_control->update_z_controller();
		break;

	case AltHold_Landed:
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors->set_desired_spool_state(
					AP_Motors::DESIRED_THROTTLE_UNLIMITED);
		}

#if FRAME_CONFIG == HELI_FRAME    
		if (heli_flags.init_targets_on_arming) {
			attitude_control->reset_rate_controller_I_terms();
			attitude_control->set_yaw_target_to_current_heading();
			if (motors->get_interlock()) {
				heli_flags.init_targets_on_arming=false;
			}
		}
#else
		attitude_control->reset_rate_controller_I_terms();
		attitude_control->set_yaw_target_to_current_heading();
#endif
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
				target_roll, target_pitch, target_yaw_rate,
				get_smoothing_gain());
		pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
		pos_control->update_z_controller();
		break;

	case AltHold_Flying:
		motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
		// apply avoidance
		copter.avoid.adjust_roll_pitch(target_roll, target_pitch,
				copter.aparm.angle_max);
#endif


		//hal.console->printf("In AltHold Loop\r\n");
		//kobi vlad update
		float dt = (AP_HAL::millis() - lastTime)*0.001f;
		bool isUpdated = copter.optflow_kv.update();
		if(isUpdated){
			lastTime = AP_HAL::millis();
		}

		if ( isUpdated && target_roll == 0 && target_pitch == 0) {
			/*
			 * Pitch forward : < 0
			 * Pitch backward : > 0
			 * Roll right : > 0
			 * Roll left : < 0
			 */
			copter.optflow_kv.get_data(optFeature);
			optFeature /= optFeature.z;

			if(first_init){
				relativeOptFeature.x = optFeature.x;
				relativeOptFeature.y = optFeature.y;
				relativeOptFeature.z = optFeature.z;
				first_init = false;
			}
			else{
				optFeature.x -=relativeOptFeature.x;
				optFeature.y -=relativeOptFeature.y;
//				optFeature.z -=relativeOptFeature.z;
			}

//			uint16_t alt = copter.rangefinder.distance_cm_orient(
//					ROTATION_PITCH_270);
			float alt = inertial_nav.get_altitude();
			//set lidar vector direction
			lidarDirection.x = 0;
			lidarDirection.y = 0;
			lidarDirection.z = alt;

			copterRotInv.from_euler(ahrs.roll, ahrs.pitch, 0);
			//copterRotInv.inverse().rotate_vector_by_quaternion(lidarDirection);

			optFeature *= (ref_alt-alt);
			copterRotInv.rotate_vector_by_quaternion(optFeature);

			//hal.console->printf("V : (%.2f,%.2f,%.2f),%.2f\r\n",optFeature.x,optFeature.y,optFeature.z,lidarDirection.z);
			//hal.console->printf("V : (%.2f,%.2f,%.2f)\r\n",lidarDirection.x,lidarDirection.y,lidarDirection.z);

//			kf_2d_opicalflow.update_measurment_x(optFeature.x,0,0,dt);
//			kf_2d_opicalflow.update_measurment_x(optFeature.y,0,0,dt);
//			kf_2d_opicalflow.update_predication(dt);
//
//			kf_2d_opicalflow.get_prediction_x(optFeature.x,temp,temp);
//			kf_2d_opicalflow.get_prediction_y(optFeature.y,temp,temp);

//			hal.console->printf("fea : (%.2f,%.2f,%.2f)\r\n", optFeature.x,optFeature.y,lidarDirection.z);

			Vector3f diff = optFeature - oldOptFeature;

			float d_roll = diff.y / dt;
			float d_pitch = diff.x / dt;

			roll_err += optFeature.y * dt;
			pitch_err += optFeature.x * dt;
			roll_err = constrain_float(roll_err, -i_flow_max, i_flow_max);
			pitch_err = constrain_float(pitch_err, -i_flow_max, i_flow_max);

			roll_out = optFeature.y * p_flow + d_roll * d_flow + roll_err * i_flow;
			pitch_out = -optFeature.x * p_flow - d_pitch * d_flow - pitch_err * i_flow;

			oldOptFeature.x = optFeature.x;
			oldOptFeature.y = optFeature.y;
			oldOptFeature.z = optFeature.z;

//			hal.console->printf("fea : (%.2f,%.2f)\r\n", optFeature.x,optFeature.y);
//			hal.console->printf("err : (%.2f,%.2f)\r\n", roll_err, pitch_err);
//			hal.console->printf("Target : (%.2f,%.2f),(%.2f,%.2f)\r\n", target_roll,target_pitch,d_roll,d_pitch);

			roll_out = constrain_float(roll_out, -1000, 1000);
			pitch_out = constrain_float(pitch_out, -1000, 1000);
			/*
			 * Pitch forward : < 0
			 * Pitch backward : > 0
			 * Roll right : > 0
			 * Roll left : < 0
			 */
			//hal.console->printf("(%.2f,%.2f)\r\n", roll_out,pitch_out);
			//hal.console->printf("(%.2f,%.2f)\r\n", roll_err* i_flow,pitch_err*i_flow);

			copter.Log_Write_OPT_PI(dt*1000.0f,optFeature.x,optFeature.y,optFeature.z,pitch_err,roll_err,d_pitch,d_roll);

		} else if (target_roll != 0 || target_pitch != 0) {
			//}else{
			optFeature.x = 0;
			optFeature.y = 0;
			optFeature.z = 0;
			oldOptFeature.x = 0;
			oldOptFeature.y = 0;
			oldOptFeature.z = 0;
			roll_err = 0;
			pitch_err = 0;
			roll_out = target_roll;
			pitch_out = target_pitch;
			//ahrs.p_flow,ahrs.d_flow;
		}
//		copter.Log_Write_Optflow();
		//vk_opt
		copter.Log_Write_PID_OPT(roll_out,pitch_out);
//		hal.console->printf("Flow : (%.2f,%.2f)\r\n", ahrs.p_flow.cast_to_float(), ahrs.d_flow.cast_to_float());
//		hal.console->printf("T : (%.2f,%.2f)\r\n", roll_out, pitch_out);
//		hal.console->printf("O : (%.2f,%.2f,%.2f)\r\n", optFeature.x, optFeature.y,optFeature.z);

		// call attitude controller
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_out, pitch_out, target_yaw_rate, get_smoothing_gain());
		//attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate,get_smoothing_gain());
		// adjust climb rate using rangefinder
		if (copter.rangefinder_alt_ok()) {
			// if rangefinder is ok, use surface tracking
			target_climb_rate = get_surface_tracking_climb_rate(
					target_climb_rate, pos_control->get_alt_target(), G_Dt);
		}

		// get avoidance adjusted climb rate
		target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt,
				false);
		pos_control->update_z_controller();
		break;
	}
}
