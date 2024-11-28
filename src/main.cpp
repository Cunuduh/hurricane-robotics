#include "main.h"
pros::Optical colour_sensor(12);
pros::Rotation lb_rotation(8);
pros::MotorGroup intake({-21, -7}, pros::v5::MotorGears::green);
std::atomic<int> intake_power{0};
std::atomic<bool> intake_running{false};
std::atomic<bool> colour_rejection_active{false};
std::atomic<int> last_rejection_time{0};
pros::Motor lb(13, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
std::shared_ptr<okapi::ChassisController> chassis =
	okapi::ChassisControllerBuilder()
		.withMotors({-1, -3, 5}, {2, 4, -6})
		.withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in, 15_in}, okapi::imev5BlueTPR})
		.build();
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-1, -3, 5}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({2, 4, -6}, pros::v5::MotorGears::blue);
enum class colour
{
	RED,
	BLUE,
	NONE
};
colour team_colour = colour::BLUE;
colour detect_colour()
{
	double hue = colour_sensor.get_hue();
	if (colour_sensor.get_proximity() > 128)
	{
		master.print(1, 0, "None");
		return colour::NONE;
	}
	// 330-30 degrees = red
	if (hue >= 330.0 || hue <= 30.0)
	{
		master.print(1, 0, "Red ");
		return colour::RED;
	}
	// 150-210 degrees = blue
	else if (hue >= 150.0 && hue <= 210.0)
	{
		master.print(1, 0, "Blue");
		return colour::BLUE;
	}
	else
	{
		master.print(1, 0, "None");
		return colour::NONE;
	}
}
void initialize()
{
	pros::lcd::initialize();
	lb_rotation.reset_position();
	std::string team = (team_colour == colour::RED) ? "Red" : "Blue";
	master.print(0, 0, ("Team:" + team).c_str());
	master.print(1, 0, "None");
	master.print(2, 0, "FilterOff");
	intake_power = 0;
	intake_running = false;
	colour_rejection_active = false;
	pros::Task colour_rejection_task{[&]
	{
		while (true)
		{
			if (intake_power != 0)
			{
				colour detected = detect_colour();
				if (detected != colour::NONE && detected != team_colour)
				{
					colour_rejection_active = true;
					last_rejection_time = pros::millis();
					intake.move(0);
				}
				if (colour_rejection_active && (pros::millis() - last_rejection_time > 500))
				{
					colour_rejection_active = false;
					intake.move(intake_power);
				}
			}
			pros::delay(30);
		}
	}};
}
void disabled() {}
void competition_initialize() {}
void activate_intake(int duration_ms = 0, int power = 127)
{
	intake.move(power);
	if (duration_ms > 0)
	{
		pros::delay(duration_ms);
		intake.move(0);
	}
}
void activate_lb(int duration_ms = 0)
{
	lb.move(127);
	pros::delay(duration_ms);
	lb.move(-127);
	pros::delay(duration_ms);
	lb.move(0);
}
void autonomous()
{
	chassis->setMaxVelocity(250);
	solenoid.set_value(false);
	chassis->moveDistance(-1.25_ft);
	chassis->turnAngle(45_deg);
	chassis->setMaxVelocity(225);
	chassis->moveDistance(-4_ft);
	solenoid.set_value(true);
	chassis->moveDistance(-0.5_ft);
	chassis->setMaxVelocity(250);
	chassis->turnAngle(75_deg);
	activate_intake();
	chassis->moveDistance(3.5_ft);
	pros::delay(1000);
	activate_intake(0, 0);
	chassis->moveDistance(-1.5_ft);
	chassis->turnAngle(125_deg); // from 120
	chassis->setMaxVelocity(75);
	activate_intake();
	chassis->moveDistance(1.5_ft);
	pros::delay(2000);
	activate_intake(0);
}
void autonomous_alt() {
	chassis->moveDistance(-1_ft);
	chassis->turnAngle(-45_deg);
	chassis->moveDistance(-1.5_ft);
	activate_intake(1000);
	activate_lb(500);
	chassis->turnAngle(67.5_deg);
	chassis->moveDistance(-4.5_ft);
	chassis->turnAngle(-112.5_deg);
	pros::delay(500);
	solenoid.set_value(true);
	chassis->moveDistance(2_ft);
	activate_intake(2000);
	chassis->turnAngle(90_deg);
	chassis->moveDistance(2_ft);
	activate_intake(2000);
	chassis->turnAngle(-45_deg);
	chassis->moveDistance(3_ft);
	solenoid.set_value(false);
	chassis->moveDistance(8.5_ft);
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
	static bool override_active = false;
	static bool solenoid_state = false;
	while (true)
	{
		int left_input = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int right_input = quad_curve(master.get_analog(ANALOG_RIGHT_Y));
		if (master.get_digital_new_press(DIGITAL_X))
		{
			override_active = !override_active;
			if (override_active)
			{
				master.print(2, 0, "FilterOff");
			}
			else
			{
				master.print(2, 0, "FilterOn ");
			}
		}
		if (master.get_digital_new_press(DIGITAL_A))
		{
			autonomous();
		}
		if (master.get_digital_new_press(DIGITAL_B))
		{
			autonomous_alt();
		}
		if (master.get_digital(DIGITAL_RIGHT))
		{
			lb.move(127);
		}
		else if (master.get_digital(DIGITAL_LEFT))
		{
			lb.move(-127);
		}
		else
		{
			lb.move(0);
		}
		if (master.get_digital_new_press(DIGITAL_L2))
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
		if (!colour_rejection_active || override_active)
		{
			intake.move(intake_power);
		}

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