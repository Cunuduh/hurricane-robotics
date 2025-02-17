#include "main.h"
double flip = 1.0;
pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
auto imu = std::make_shared<okapi::IMU>(7);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
pros::Motor intake_conveyor(8, pros::v5::MotorGears::green);
pros::Motor lb(10, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
pros::ADIDigitalOut doinker('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-4, -5, 6}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({1, 2, -3}, pros::v5::MotorGears::blue);
std::atomic<int32_t> intake_power{0};
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
	intake_power = rpm;
	intake.move_velocity(intake_power);
	if (duration_ms > 0)
		pros::delay(duration_ms);
	else return;
	intake.move_velocity(0);
}

void activate_lb(int duration_ms = 0)
{
	lb.move_velocity(200);
	pros::delay(duration_ms);
	lb.move_velocity(0);
}

void turn(std::shared_ptr<okapi::ChassisController> chassis, okapi::QAngle angle)
{
	double initial_velocity = chassis->getMaxVelocity();
	chassis->setMaxVelocity(125);
	chassis->turnAngle(angle);
	chassis->setMaxVelocity(initial_velocity);
}
void activate_sol(bool value)
{
	solenoid_on = value;
	solenoid.set_value(solenoid_on);
}

bool is_intake_stalled(const pros::MotorGroup& motors, int threshold = 60)
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
void autonomous_normal_p()
{
	pros::Task intake_task{[&]{
    while (true)
		{
      if (is_intake_stalled(intake))
			{
        attempt_unjam();
      }
      pros::delay(100);
    }
	}};
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
				{0.0040, 0.000, 0.00003}, // Distance controller gains
				{0.0050, 0.000, 0.0000025}, // Turn controller gains
				{0.0003, 0.000, 0.000025}  // Angle controller gains (helps drive straight)
			)
			.build();
	if (!pros::competition::is_autonomous())
	{
		imu->calibrate();
	}
	chassis->setMaxVelocity(263);
	activate_lb(400);
	
	chassis->moveDistance(-1.425_ft);
	turn(chassis, 30_deg * flip);
	chassis->moveDistanceAsync(-2.5_ft);
	pros::delay(500);
	solenoid.set_value(true);
	pros::delay(500);
	intake_power = 200;
	intake.move_velocity(200);
	turn(chassis, -120_deg * flip);
	chassis->moveDistance(2.5_ft); // score first and second rings
	pros::delay(5000);
	intake_power = 0;
	intake.move_velocity(0);
}
void autonomous_normal_n()
{
	pros::Task intake_task{[&]{
    while (true)
		{
      if (is_intake_stalled(intake))
			{
        attempt_unjam();
      }
      pros::delay(100);
    }
	}};
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
				{0.0040, 0.000, 0.00003}, // Distance controller gains
				{0.0050, 0.000, 0.0000025}, // Turn controller gains
				{0.0003, 0.000, 0.000025}  // Angle controller gains (helps drive straight)
			)
			.build();
	if (!pros::competition::is_autonomous())
	{
		imu->calibrate();
	}
	chassis->setMaxVelocity(263);
	activate_lb(400);

	chassis->moveDistance(-1.425_ft);
	turn(chassis, 30_deg * flip);
	chassis->moveDistanceAsync(-2.55_ft);
	pros::delay(500);
	solenoid.set_value(true);
	pros::delay(500);
	intake_power = 200;
	intake.move_velocity(200);
	turn(chassis, 60_deg * flip);
	chassis->moveDistance(2_ft);
	turn(chassis, 75_deg * flip);
	chassis->setMaxVelocity(100);
	chassis->moveDistance(1.125_ft);
	chassis->moveDistance(-1.125_ft);
	pros::delay(5000);
	intake_power = 0;
	intake.move_velocity(0);
}
void autonomous_skills()
{
	pros::Task intake_task{[&]{
		pros::delay(3000);
    while (true)
		{
      if (is_intake_stalled(intake))
			{
        attempt_unjam();
      }
      pros::delay(100);
    }
	}};
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
				{0.0040, 0.000, 0.00003}, // Distance controller gains
				{0.0050, 0.000, 0.0000025}, // Turn controller gains
				{0.0003, 0.000, 0.000025}  // Angle controller gains (helps drive straight)
			)
			.build();
	if (!pros::competition::is_autonomous())
	{
		imu->calibrate();
	}
	chassis->setMaxVelocity(263);
	activate_lb(400);

	activate_intake(2000);
	chassis->moveDistance(1.25_ft);
	turn(chassis, 90_deg);
	chassis->moveDistanceAsync(-2.45_ft);
	pros::delay(500);
	solenoid.set_value(true);
	pros::delay(500);
	intake_power = 200;
	intake.move_velocity(200);
	turn(chassis, -90_deg);
	chassis->moveDistance(2_ft); // pick up first ring
	turn(chassis, -90_deg); // originally 95
	chassis->moveDistance(2_ft); // pick up second ring
	turn(chassis, 65_deg);
	chassis->moveDistance(1.6_ft); // pick up third ring
	chassis->moveDistance(-1.65_ft);
	turn(chassis, -160_deg); // originally 155
	chassis->moveDistance(3.25_ft); // pick up fourth and fifth rings
	chassis->moveDistance(-1.25_ft);
	turn(chassis, 90_deg);
	chassis->moveDistanceAsync(1.5_ft); // pick up sixth ring
	pros::delay(1500);
	if (!chassis->isSettled())
	{
		chassis->stop();
	}
	chassis->moveDistance(-1.5_ft);
	turn(chassis, 135_deg);
	chassis->moveDistanceAsync(-1.75_ft); // back up into corner and drop mobile goal
	pros::delay(1000);
	if (!chassis->isSettled())
	{
		chassis->stop();
	}
	activate_sol(false);
	intake_power = 0;
	intake.move_velocity(0);
	chassis->moveDistance(1.75_ft);
	turn(chassis, -135_deg); // originally 140
	chassis->moveDistance(-5_ft);

	// part 2
	chassis->moveDistanceAsync(-1.5_ft);
	pros::delay(500);
	activate_sol(true);
	pros::delay(500);
	intake_power = 200;
	intake.move_velocity(200);
}

void autonomous()
{
	autonomous_normal_n();
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
		bool l1_held = master.get_digital(DIGITAL_L1);
		bool r1_press = master.get_digital_new_press(DIGITAL_R1);
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

		//if (y_press && !pros::competition::is_connected())
		//	autonomous();

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
		else
		{
			intake_power = 0;
		}
		if (l1_held)
		{
			intake_power = -200;
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

		if (right_held && (get_lb_angle() < 110.0 || get_lb_angle() > 330.0))
		{
			lb.move_velocity(100);
		}
		else if (left_held)
		{
			lb.move_velocity(-100);
		}
		else
		{
			lb.move_velocity(0);
		}

		intake.move_velocity(intake_power);
		left_motors.move_velocity(left_input);
		right_motors.move_velocity(right_input);
		pros::delay(3);
	}
}