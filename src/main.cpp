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
bool climb_on = false;

double lb_presets[] = {5.0, 17.0, 110.0};
size_t lb_preset_index = 0;
bool moving_lb = false;
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
	pros::delay(duration_ms);
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
	chassis->moveDistance(-3_ft); // pick up first mobile goal
	activate_sol();
	chassis->turnAngle(180_deg);
	activate_intake();
	chassis->moveDistance(4_ft);
	chassis->moveDistance(-2_ft);
	chassis->turnAngle(-90_deg);
	chassis->moveDistance(2_ft); // nab three rings top left corner
	chassis->moveDistance(-1_ft);
	chassis->turnAngle(180_deg);
	chassis->moveDistance(3.25_ft);
	chassis->turnAngle(22.5_deg);
	chassis->moveDistance(1.8_ft);
	chassis->turnAngle(167_deg);
	chassis->moveDistance(3.75_ft);
	chassis->turnAngle(-105_deg);
	chassis->moveDistance(-5.7_ft);
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
		bool a_press = master.get_digital_new_press(DIGITAL_A);
		bool x_press = master.get_digital_new_press(DIGITAL_X);
		int32_t analog_left_y = master.get_analog(ANALOG_LEFT_Y);
		int32_t analog_right_x = master.get_analog(ANALOG_RIGHT_X);

		int32_t power = cube_curve(analog_left_y, 600);
		int32_t turn = cube_curve(analog_right_x, 600);
		int32_t left_input = power + turn;
		int32_t right_input = power - turn;

		if (master.get_digital_new_press(DIGITAL_Y) && !pros::competition::is_connected())
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
			if (master.get_digital(DIGITAL_L1))
			{
				intake_power = -200;
			}
			else
			{
				intake_power = 200;
			}
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

		if (master.get_digital_new_press(DIGITAL_LEFT))
		{
			if (lb_preset_index > 0)
			{
				lb_preset_index--;
				moving_lb = true;
			}
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT)) 
		{
			if (lb_preset_index < 2)
			{
				lb_preset_index++;
				moving_lb = true;
			}
		}

		if (master.get_digital(DIGITAL_UP))
		{
			lb.move_velocity(100);
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			lb.move_velocity(-100);
		}
		else if (moving_lb)
		{
			double current_angle = get_lb_angle();
			double target_angle = lb_presets[lb_preset_index];
			double error = target_angle - current_angle;
			
			if (std::fabs(error) > 5)
			{
				int vel = static_cast<int>(error * 2);
				if (vel > 100)
				{
					vel = 100;
				}
				if (vel < -100)
				{
					vel = -100;
				}
				lb.move_velocity(vel);
			}
			else
			{
				lb.move_velocity(0);
				moving_lb = false;
			}
		}
		else
		{
			lb.move_velocity(0);
		}

		intake.move_velocity(intake_power);
		left_motors.move_velocity(left_input);
		right_motors.move_velocity(right_input);
		pros::lcd::print(1, "LB: %.2f", get_lb_angle());
		pros::lcd::print(2, "LB Preset: %d ", lb_preset_index);
		pros::delay(3);
	}
}