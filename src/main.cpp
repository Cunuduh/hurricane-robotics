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

	while (true) {
		int left_input = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int right_input = quad_curve(master.get_analog(ANALOG_RIGHT_Y));
		int intake_input = master.get_digital(DIGITAL_L2) - master.get_digital(DIGITAL_R2);
		int left_speed = decelerate(left_input, prev_left);
		int right_speed = decelerate(right_input, prev_right);

		prev_left = left_speed;
		prev_right = right_speed;

		left_motors.move(left_speed);
		right_motors.move(right_speed);
		intake.move(intake_input * 127);
		pros::delay(5);
	}
}