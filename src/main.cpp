#include "main.h"
pros::Optical colour_sensor(12);
pros::Rotation lb_rotation(8);
std::shared_ptr<okapi::ContinuousRotarySensor> imu = std::make_shared<okapi::IMU>(20);
pros::MotorGroup intake({-21, -7}, pros::v5::MotorGears::green);
std::atomic<int> intake_power{0};
std::atomic<bool> intake_running{false};
std::atomic<bool> colour_rejection_active{false};
std::atomic<int> last_rejection_time{0};
pros::Motor lb(13, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
pros::ADIDigitalOut doinker('B');
pros::ADIDigitalOut climb('C');
std::shared_ptr<okapi::OdomChassisController> chassis =
	okapi::ChassisControllerBuilder()
		.withMotors({-1, -3, 5}, {2, 4, -6})
		.withDimensions({okapi::AbstractMotor::gearset::blue, (72.0 / 48.0)}, {{3.25_in, 15.5_in}, okapi::imev5BlueTPR})
		.withSensors(
			std::make_shared<okapi::IntegratedEncoder>(1, true),
			std::make_shared<okapi::IntegratedEncoder>(2, false),
			imu
		)
		.withOdometry()
		.buildOdometry();
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
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
		return;
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
bool is_intake_stalled(const pros::MotorGroup& motors, int threshold = 60)
{
	return std::abs(motors.get_actual_velocity_all()[0]) < 10 && 
				 std::abs(motors.get_target_velocity_all()[0]) > threshold;
}

void attempt_unjam(pros::MotorGroup& intake, std::shared_ptr<okapi::OdomChassisController> chassis)
{
	intake.move(-127);
	chassis->moveDistance(-3_in);
	intake.move(127);
	chassis->moveDistance(3_in);
}

void activate_intake(int duration_ms = 0, int power = 127, bool check_stall = false)
{
	intake.move(power);
	if (duration_ms > 0)
	{
		int time_elapsed = 0;
		int check_interval = duration_ms / 10;
		while (time_elapsed < duration_ms) 
		{
			if (check_stall && is_intake_stalled(intake))
			{
				attempt_unjam(intake, chassis);
				intake.move(power);
			}
			pros::delay(check_interval);
			time_elapsed += check_interval;
		}
		intake.move(0);
	}
	else if (check_stall)
	{
		while (true)
		{
			if (is_intake_stalled(intake))
			{
				attempt_unjam(intake, chassis);
				intake.move(power);
			}
				pros::delay(50);
		}
	}
}
void activate_lb(int duration_ms = 0)
{
	lb.move(100);
	pros::delay(duration_ms);
	lb.move(-100);
	pros::delay(duration_ms);
	lb.move(0);
}
void autonomous()
{
	solenoid.set_value(false);
	doinker.set_value(false);
	lb.move(64);
	pros::delay(450);
	lb.move(0);
	// chassis->moveDistance(-2.75_ft);
	// chassis->turnAngle(-180_deg); // -90
	// activate_lb(1500);
	// lb.move(100);
	// pros::delay(100);
	// lb.move(0);
	// chassis->turnAngle(180_deg); // 90
	// chassis->moveDistance(4_ft);
	// chassis->turnAngle(-180_deg); // -90
	// chassis->moveDistance(-4_ft);
	// chassis->turnAngle(180_deg); // 90
	chassis->setMaxVelocity(150);
	chassis->moveDistance(-3_ft);
	solenoid.set_value(true);
	chassis->moveDistance(-0.2_ft);
	chassis->setMaxVelocity(250);
	chassis->turnAngle(90_deg);
	activate_intake();
	chassis->moveDistance(2_ft);
	pros::delay(1500);
	activate_intake(0, 0);
	chassis->moveDistance(-0.33_ft);
	chassis->turnAngle(90_deg);
	chassis->setMaxVelocity(75);
	chassis->moveDistance(0.75_ft);
	activate_intake();
	chassis->moveDistance(0.25_ft);
	pros::delay(1000);
	activate_intake(0, 0);
	chassis->setMaxVelocity(250);
	chassis->moveDistance(-3_ft);
	chassis->setMaxVelocity(200);
	chassis->turnAngle(85_deg);
	chassis->setMaxVelocity(250);
	chassis->moveDistance(4_ft);
	activate_intake();
	chassis->moveDistance(1_ft);
	pros::delay(1500);
	activate_intake(0, 0);
	chassis->turnAngle(-90_deg);
	chassis->moveDistance(2.5_ft);
}
void autonomous_skills()
{}
void autonomous_alt()
{
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
	static bool doinker_state = false;
	static bool climb_state = false;
	while (true)
	{
		int left_input = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int right_input = quad_curve(master.get_analog(ANALOG_RIGHT_Y));
		// if (master.get_digital_new_press(DIGITAL_X))
		// {
		// 	override_active = !override_active;
		// 	if (override_active)
		// 	{
		// 		master.print(2, 0, "FilterOff");
		// 	}
		// 	else
		// 	{
		// 		master.print(2, 0, "FilterOn ");
		// 	}
		// }
		if (master.get_digital_new_press(DIGITAL_X))
		{
			climb_state = !climb_state;
			climb.set_value(climb_state);
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
			lb.move(100);
		}
		else if (master.get_digital(DIGITAL_LEFT))
		{
			lb.move(-100);
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
		if (master.get_digital_new_press(DIGITAL_Y))
		{
			doinker_state = !doinker_state;
			doinker.set_value(doinker_state);
		}
		left_motors.move(left_input);
		right_motors.move(right_input);
		if (!colour_rejection_active || override_active)
		{
			intake.move(intake_power);
		}

		pros::delay(5);
	}
}