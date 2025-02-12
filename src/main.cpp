#include "main.h"
double flip = 1.0;
pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
auto imu = std::make_shared<okapi::IMU>(7);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
int32_t intake_power{0};
bool intake_running{false};
bool colour_rejection_active{false};
int32_t last_rejection_time{0};
pros::Motor lb(10, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
pros::ADIDigitalOut doinker('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-4, -5, 6}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({1, 2, -3}, pros::v5::MotorGears::blue);
int32_t intake_power = 0;
bool intake_running = false;
bool solenoid_on = false;
bool doinker_on = false;

double get_lb_angle()
{
	return lb_rotation.get_position() / 100.0;
}
void initialize()
{
	pros::lcd::initialize();
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb_rotation.reset_position();
	intake_power = 0;
	intake_running = false;
}
void disabled() {}
void competition_initialize() {}
void activate_intake(int duration_ms = 0, int rpm = 200)
{
	intake.move_velocity(rpm);
	if (duration_ms > 0)
		pros::delay(duration_ms);
	else return;
	intake.move_velocity(0);
}

void activate_lb(int duration_ms = 0)
{
	lb.move_velocity(200);
	pros::delay(duration_ms);
	lb.move_velocity(-200);
	pros::delay(duration_ms);
	lb.move_velocity(0);
}
void autonomous_skills()
{}
void turn(std::shared_ptr<okapi::ChassisController> chassis, okapi::QAngle angle)
{
	double initial_velocity = chassis->getMaxVelocity();
	chassis->setMaxVelocity(150);
	chassis->turnAngle(angle);
	chassis->setMaxVelocity(initial_velocity);
}
void activate_sol()
{
	solenoid_on = !solenoid_on;
	solenoid.set_value(solenoid_on);
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
				200_ms
			)
			.build();
	imu->calibrate();
	chassis->setMaxVelocity(200);

	activate_intake(2000); // Deposit preload ring into alliance wall stake
	chassis->moveDistance(1.25_ft);
	chassis->turnAngle(90_deg);
	chassis->moveDistance(-2.25_ft); // pick up first mobile goal
	activate_sol();
	chassis->moveDistance(1_ft);
	chassis->moveDistance(-1_ft);
	chassis->turnAngle(-90_deg);
	activate_intake();
	chassis->moveDistance(2.25_ft); // pick up first ring
	chassis->turnAngle(-56.3_deg); // arctan(3/2)
	chassis->moveDistance(3.75_ft); // pick up second ring
	chassis->turnAngle(-146.2_deg);
	chassis->moveDistance(2.25_ft); // pick up third ring
	chassis->turnAngle(33.7_deg);
	chassis->moveDistance(3.5_ft); // pick up fourth and fifth rings
	chassis->moveDistance(-0.5_ft);
	chassis->turnAngle(135_deg);
	chassis->moveDistance(1.5_ft); // pick up sixth ring
	intake.move_velocity(0);
	chassis->turnAngle(90_deg);
	chassis->moveDistance(-2_ft); // back up into corner and drop mobile goal
	activate_sol();
	chassis->moveDistance(1.5_ft);
	chassis->turnAngle(45_deg);
	chassis->moveDistance(6_ft);
	chassis->turnAngle(180_deg);
	chassis->moveDistance(-1.5_ft);
	activate_sol();
	chassis->moveDistance(-1.5_ft);
	chassis->turnAngle(90_deg);
	activate_intake();
	chassis->moveDistance(2.25_ft);
	chassis->turnAngle(56.3_deg);
	chassis->moveDistance(3.75_ft);
	chassis->turnAngle(146.2_deg);
	chassis->moveDistance(2.25_ft);
	chassis->turnAngle(-33.7_deg);
	chassis->moveDistance(3.5_ft);
	chassis->moveDistance(-0.5_ft);
	chassis->turnAngle(-135_deg);
	chassis->moveDistance(1.5_ft);
	intake.move_velocity(0);
	chassis->turnAngle(-90_deg);
	chassis->moveDistance(-2_ft);
	activate_sol();
}
int cube_curve(int input, int max_rpm)
{
	float norm = input / 127.0f;
	float curved = norm * norm * norm;
	return static_cast<int>(curved * max_rpm);
}
void opcontrol()
{
	while (true)
	{
		bool l2_press = master.get_digital_new_press(DIGITAL_L2);
		bool r2_press = master.get_digital_new_press(DIGITAL_R2);
		bool r1_press = master.get_digital_new_press(DIGITAL_R1);
		bool l1_held = master.get_digital_new_press(DIGITAL_L1);
		bool a_press = master.get_digital_new_press(DIGITAL_A);
		bool b_press = master.get_digital_new_press(DIGITAL_B);
		bool x_press = master.get_digital_new_press(DIGITAL_X);
		bool y_press = master.get_digital_new_press(DIGITAL_Y);
		bool right_held = master.get_digital(DIGITAL_RIGHT);
		bool left_held = master.get_digital(DIGITAL_LEFT);
		int32_t analog_left_y = master.get_analog(ANALOG_LEFT_Y);
		int32_t analog_right_x = master.get_analog(ANALOG_RIGHT_X);

		int32_t power = cube_curve(analog_left_y, 600);
		int32_t turn = cube_curve(analog_right_x, 600);
		int32_t left_input = power + turn;
		int32_t right_input = power - turn;

		if (y_press && !pros::competition::is_connected())
			autonomous();

		if (l2_press)
		{
			intake_running = true;
		}
		else if (r2_press)
		{
			intake_running = false;
		}

		if (intake_running)
		{
			intake_power = 200;
		}
		else if (l1_held)
		{
			intake_power = -200;
		}
		else
		{
			intake_power = 0;
		}
		if (r1_press)
		{
			solenoid_on = !solenoid_on;
			solenoid.set_value(solenoid_on);
		}
		if (a_press)
		{
			doinker_on = !doinker_on;
			doinker.set_value(doinker_on);
		}

		if (right_held)
		{
			lb.move_velocity(100);
		}
		else if (left_held)
		{
			lb.move_velocity(-100);
		}

		intake.move_velocity(intake_power);
		left_motors.move_velocity(left_input);
		right_motors.move_velocity(right_input);
		pros::delay(3);
	}
}