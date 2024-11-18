#include "main.h"

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}
void disabled() {}
void competition_initialize() {}
void autonomous()
{
	pros::Motor intake(21, pros::v5::MotorGears::green);
	pros::ADIDigitalOut solenoid('A');
	std::shared_ptr<okapi::ChassisController> chassis =
		okapi::ChassisControllerBuilder()
			.withMotors({-1, -3, 5}, {2, 4, -6})
			.withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in, 15_in}, okapi::imev5BlueTPR})
			.build();

	chassis->moveDistance(-0.5_ft);
	chassis->turnAngle(45_deg);
	pros::delay(100);
	chassis->moveDistance(-3.1_ft); // sqrt (2^2 + 2^2) = 2.83
	solenoid.set_value(true);
	chassis->turnAngle(45_deg);
	intake.move(127);
	pros::delay(100);
	chassis->moveDistance(3_ft);
	intake.move(0);
}
int quad_curve(int input)
{
	float norm = input / 127.0f;
	int sign = (norm >= 0) ? 1 : -1;
	float curved = norm * norm * sign;
	return static_cast<int>(curved * 127.0f);
}
void opcontrol()
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_motors({-1, -3, 5}, pros::v5::MotorGears::blue);
	pros::MotorGroup right_motors({2, 4, -6}, pros::v5::MotorGears::blue);
	pros::Motor intake(21, pros::v5::MotorGears::green);
	pros::ADIDigitalOut solenoid('A');
	static int intake_power = 0;
	static bool intake_running = false;
	static bool solenoid_state = false;

	while (true)
	{
		int left_input = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int right_input = quad_curve(master.get_analog(ANALOG_RIGHT_Y));
		if (master.get_digital_new_press(DIGITAL_A))
		{
			autonomous();
		}
		if (master.get_digital_new_press(DIGITAL_L2) && !solenoid_state)
		{
			intake_running = true;
			intake_power = 127;
		}
		if (master.get_digital_new_press(DIGITAL_R2))
		{
			intake_running = false;
			intake_power = 0;
		}
		if (master.get_digital(DIGITAL_L1))
		{
			intake_power = -127;
		}
		else
		{
			intake_power = intake_running ? 127 : 0;
		}
		if (master.get_digital_new_press(DIGITAL_R1))
		{
			solenoid_state = !solenoid_state;
			solenoid.set_value(solenoid_state);
		}

		left_motors.move(left_input);
		right_motors.move(right_input);
		intake.move(intake_power);

		std::vector<double> left_temp = left_motors.get_temperature_all();
		std::vector<double> right_temps = right_motors.get_temperature_all();
		double left_temp_avg = std::accumulate(left_temp.begin(), left_temp.end(), 0.0) / left_temp.size();
		double right_temp_avg = std::accumulate(right_temps.begin(), right_temps.end(), 0.0) / right_temps.size();
		double intake_temp = intake.get_temperature();

		int battery_voltage = pros::battery::get_voltage();
		int battery_current = pros::battery::get_current();

		pros::lcd::set_text(2, "Intake Temp: " + std::to_string(intake_temp));
		pros::lcd::set_text(3, "Average Temp: " + std::to_string((left_temp_avg + right_temp_avg + intake_temp) / 3));
		pros::lcd::set_text(4, "Battery V: " + std::to_string(battery_voltage));
		pros::lcd::set_text(5, "Battery I: " + std::to_string(battery_current));
		pros::lcd::set_text(6, "Battery P: " + std::to_string(battery_voltage * battery_current));
		pros::delay(5);
	}
}