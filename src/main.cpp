#include "main.h"
double flip = 1.0;
pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
auto imu = std::make_shared<okapi::IMU>(7);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
std::atomic<int> intake_power{0};
std::atomic<bool> intake_running{false};
std::atomic<bool> colour_rejection_active{false};
std::atomic<int> last_rejection_time{0};
pros::Motor lb(10, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-4, -5, 6}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({1, 2, -3}, pros::v5::MotorGears::blue);
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
void activate_intake(int duration_ms = 0, int power = 127)
{
	intake.move(power);
	pros::delay(duration_ms);
	intake.move(0);
}
void activate_lb(int duration_ms = 0)
{
	lb.move(100);
	pros::delay(duration_ms);
	lb.move(-100);
	pros::delay(duration_ms);
	lb.move(0);
}// void autonomous_()
// {
// 	// IF PLAYING ON LEFT AUDIENCE SIDE SET FLIP TO -1
// 	solenoid.set_value(false);
// 	lb.move(127);
// 	pros::delay(300);
// 	lb.move(0);
// 	chassis->setMaxVelocity(200);
// 	chassis->moveDistance(-1.125_ft);
// 	chassis->turnAngle(-40_deg * flip);
// 	chassis->moveDistanceAsync(-2.95_ft);
// 	for (int i = 0; i < 7; i++)
// 	{
// 		chassis->setMaxVelocity(200 - i * 20);
// 		pros::delay(100);
// 	}
// 	chassis->moveDistance(-0.86_ft);
// 	solenoid.set_value(true);
// 	pros::delay(350);
// 	chassis->turnAngle(-45_deg * flip);
// 	activate_intake();
// 	chassis->setMaxVelocity(300);
// 	chassis->moveDistance(1_ft);
// 	chassis->setMaxVelocity(150);
// 	chassis->turnAngle(-90_deg * flip);
// 	chassis->moveDistance(1.25_ft);
// 	pros::delay(1000);
// 	chassis->setMaxVelocity(400);
// 	chassis->moveDistance(-0.75_ft);
// 	chassis->setMaxVelocity(250);
// 	chassis->turnAngle(-85_deg * flip);
// 	chassis->setMaxVelocity(400);
// 	chassis->moveDistance(2_ft);
// 	activate_intake(0, 0);
// 	// doinker.set_value(true);
// 	// chassis->setMaxVelocity(250);
// 	// chassis->turnAngle(-90_deg * flip);
// 	// doinker.set_value(false);
// 	// chassis->turnAngle(45_deg * flip);
// 	// activate_intake();
// 	// chassis->setMaxVelocity(450);
// 	// chassis->moveDistance(1_ft);
// 	// pros::delay(1000);
// 	// activate_intake(0, 0);
// 	// chassis->turnAngle(-45_deg * flip);
// 	// chassis->moveDistance(2.5_ft);
// }

void autonomous_skills()
{}
void turn(std::shared_ptr<okapi::ChassisController> chassis, okapi::QAngle angle)
{
	double initial_velocity = chassis->getMaxVelocity();
	chassis->setMaxVelocity(200);
	chassis->turnAngle(angle);
	chassis->setMaxVelocity(initial_velocity);
}
void autonomous()
{
	std::shared_ptr<okapi::ChassisController> chassis =
		okapi::ChassisControllerBuilder()
			.withMotors({-4, -5, 6}, {1, 2, -3})
			.withDimensions({okapi::AbstractMotor::gearset::blue, (72.0 / 48.0)}, {{3.25_in, 15.5_in}, okapi::imev5BlueTPR})
			.withSensors(
				std::make_shared<okapi::IntegratedEncoder>(6, false),
				std::make_shared<okapi::IntegratedEncoder>(3, true),
				imu
			)
			.withGains(
				{0.0040, 0.000, 0.000025}, // Distance controller gains
				{0.0045, 0.002, 0.000050}, // Turn controller gains
				{0.0003, 0.000, 0.000025}  // Angle controller gains (helps drive straight)
			)
			.withClosedLoopControllerTimeUtil(
				10.0,
				2.5,
				300_ms
			)
			.build();
	imu->calibrate();
	// go in infinite square
	chassis->setMaxVelocity(300);
	while (true)
	{
		chassis->moveDistance(2_ft);
		turn(chassis, 90_deg);
	}
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
		int power = quad_curve(master.get_analog(ANALOG_LEFT_Y));
		int turn = quad_curve(master.get_analog(ANALOG_RIGHT_X));
		int left_input = power + turn;
		int right_input = power - turn;
		if (master.get_digital_new_press(DIGITAL_A))
		{
			autonomous();
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
		left_motors.move(left_input);
		right_motors.move(right_input);
		if (!colour_rejection_active || override_active)
		{
			intake.move(intake_power);
		}
		pros::delay(5);
	}
}