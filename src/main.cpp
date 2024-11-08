#include "main.h"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}
void disabled() {}
void competition_initialize() {}
void autonomous() {}

int quad_curve(int input) {
	float norm = input / 127.0f;
	int sign = (norm >= 0) ? 1 : -1;
	float curved = norm * norm * sign;
	return static_cast<int>(curved * 127.0f);
}
int decelerate(int current, int prev) {
	static int brake_dir = 0;
	if (!current) {
		int brake_force = std::abs(prev);
		if (!brake_dir) {
			brake_dir = (prev > 0) ? -1 : 1;
		}
		if (brake_force > 64) {
			return brake_dir * (brake_force - 8);
		}
		else if (brake_force > 16) {
			return brake_dir * (brake_force / 2);
		}
		brake_dir = 0;
		return 0;
	}
	brake_dir = 0;
	return current;
}
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_motors({-1, -3, 5}, pros::v5::MotorGears::blue);
	pros::MotorGroup right_motors({2, 4, -6}, pros::v5::MotorGears::blue);
	pros::Motor intake(-7, pros::v5::MotorGears::green);
	static int prev_left = 0;
	static int prev_right = 0;
	static bool intake_running = false;

	while (true) {
		int left_input = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int right_input = quad_curve(master.get_analog(ANALOG_RIGHT_Y));
		
		if (master.get_digital_new_press(DIGITAL_L2)) {
			intake_running = true;
		}
		if (master.get_digital_new_press(DIGITAL_R2)) {
			intake_running = false;
		}
		
		int intake_power = 0;
		if (master.get_digital(DIGITAL_R1)) {
			intake_power = -127;
		} else if (intake_running) {
			intake_power = 127;
		}

		int left_speed = decelerate(left_input, prev_left);
		int right_speed = decelerate(right_input, prev_right);

		prev_left = left_speed;
		prev_right = right_speed;

		left_motors.move(left_speed);
		right_motors.move(right_speed);
		intake.move(intake_power);

		std::vector<double> left_temp = left_motors.get_temperature_all();
		std::vector<double> right_temps = right_motors.get_temperature_all();
		double left_temp_avg = std::accumulate(left_temp.begin(), left_temp.end(), 0.0) / left_temp.size();
		double right_temp_avg = std::accumulate(right_temps.begin(), right_temps.end(), 0.0) / right_temps.size();
		double intake_temp = intake.get_temperature();

		int battery_voltage = pros::battery::get_voltage();
		int battery_current = pros::battery::get_current();

		pros::lcd::set_text(3, "Average Temp: " + std::to_string((left_temp_avg + right_temp_avg + intake_temp) / 3));
		pros::lcd::set_text(4, "Battery V: " + std::to_string(battery_voltage));
		pros::lcd::set_text(5, "Battery I: " + std::to_string(battery_current));
		pros::lcd::set_text(6, "Battery P: " + std::to_string(battery_voltage * battery_current));
		pros::delay(5);
	}
}