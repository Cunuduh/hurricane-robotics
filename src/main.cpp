#include "main.h"
double flip = 1.0;
ez::Drive chassis(
		{6, -5, -4},
		{-3, 2, 1},
		7,
		3.25,
		400);

double get_lb_angle()
{
	return lb_rotation.get_position() / 100.0;
}
void initialize()
{
	default_constants();
	chassis.opcontrol_curve_default_set(10.0, 10.0);

	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb_rotation.reset_position();
	intake_power = 0;
	intake_running = false;

	pros::Task intake_task{[&]
	{
		while (true)
		{
			if (!pros::competition::is_autonomous())
				return;
			if (is_intake_stalled(intake))
				attempt_unjam();
			pros::delay(100);
		}
	}};
	ez::as::auton_selector.autons_add({{"Normal Negative", normal_n},
																		 {"Normal Positive", normal_p},
																		 {"Skills", skills}});
	chassis.initialize();
	ez::as::initialize();
}
void disabled() {}
void competition_initialize() {}
void activate_intake(int duration_ms = 0, int rpm = 200)
{
	intake_power = rpm;
	intake.move_velocity(intake_power);
	if (duration_ms > 0)
		pros::delay(duration_ms);
	else
		return;
	intake.move_velocity(0);
}

void activate_lb(int duration_ms = 0)
{
	lb.move_velocity(200);
	pros::delay(duration_ms);
	lb.move_velocity(0);
}

void activate_sol(bool value)
{
	solenoid_on = value;
	solenoid.set_value(solenoid_on);
}

bool is_intake_stalled(const pros::MotorGroup &motors, int threshold = 60)
{
	return std::abs(motors.get_actual_velocity_all()[0]) < 10 &&
				 std::abs(motors.get_target_velocity_all()[0]) > threshold;
}

void attempt_unjam()
{
	intake_conveyor.move_velocity(-200);
	pros::delay(500);
	intake_conveyor.move_velocity(intake_power);
}

void autonomous()
{
	chassis.pid_targets_reset();							 // Resets PID targets to 0
	chassis.drive_imu_reset();								 // Reset gyro position to 0
	chassis.drive_sensor_reset();							 // Reset drive sensors to 0
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

	ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
}
void ez_template_extras()
{
	// Only run this when not connected to a competition switch
	if (!pros::competition::is_connected())
	{
		// PID Tuner
		// - after you find values that you're happy with, you'll have to set them in auton.cpp

		// Enable / Disable PID Tuner
		//  When enabled:
		//  * use A and Y to increment / decrement the constants
		//  * use the arrow keys to navigate the constants
		if (master.get_digital_new_press(DIGITAL_X))
			chassis.pid_tuner_toggle();

		// Trigger the selected autonomous routine
		if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN))
		{
			pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
			autonomous();
			chassis.drive_brake_set(preference);
		}

		// Allow PID Tuner to iterate
		chassis.pid_tuner_iterate();
	}

	// Disable PID Tuner when connected to a comp switch
	else
	{
		if (chassis.pid_tuner_enabled())
			chassis.pid_tuner_disable();
	}
}

void opcontrol()
{
	chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
	while (true)
	{
		ez_template_extras();
		chassis.opcontrol_arcade_standard(ez::SPLIT);

		if (master.get_digital_new_press(DIGITAL_L2))
			intake_running = true;
		else if (master.get_digital_new_press(DIGITAL_R2))
			intake_running = false;

		if (intake_running)
			intake_power = 200;
		else
			intake_power = 0;

		if (master.get_digital(DIGITAL_L1))
			intake_power = -200;
		if (master.get_digital_new_press(DIGITAL_R1))
		{
			solenoid_on = !solenoid_on;
			solenoid.set_value(solenoid_on);
		}
		if (master.get_digital_new_press(DIGITAL_A))
		{
			doinker_on = !doinker_on;
			doinker.set_value(doinker_on);
		}

		if (master.get_digital_new_press(DIGITAL_RIGHT) && (get_lb_angle() < 110.0 || get_lb_angle() > 330.0))
			lb.move_velocity(100);
		else if (master.get_digital_new_press(DIGITAL_LEFT))
			lb.move_velocity(-100);
		else
			lb.move_velocity(0);

		intake.move_velocity(intake_power);
		pros::delay(ez::util::DELAY_TIME);
	}
}