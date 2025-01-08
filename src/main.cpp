#include "main.h"
double flip = 1.0;
pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
auto imu = std::make_shared<okapi::IMU>(7);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
int intake_power = 0;
bool intake_running = false;
pros::Motor lb(10, pros::v5::MotorGears::green);
pros::ADIDigitalOut solenoid('A');
pros::ADIDigitalOut doinker('B');
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-4, -5, 6}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({1, 2, -3}, pros::v5::MotorGears::blue);
struct record_frame
{
	int left_input;
	int right_input;
	int lb_input;
	int intake_power;
	bool solenoid_state;
	bool doinker_state;
};
bool recording = false;
std::vector<record_frame> frames;
void initialize()
{
	pros::lcd::initialize();
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lb_rotation.reset_position();
	intake_power = 0;
	intake_running = false;
	std::ifstream file("/usd/recorded.txt");
	frames.reserve(12000);
	if (file.is_open())
	{
		frames.clear();
		record_frame frame;
		while (file >> frame.left_input >> frame.right_input >> frame.lb_input >> frame.intake_power >> frame.solenoid_state >> frame.doinker_state)
		{
			frames.push_back(frame);
		}
		file.close();
		pros::lcd::print(0, "Loaded %d frames", frames.size());
	}
}
void disabled() {}
void competition_initialize() {}
void activate_intake(int duration_ms = 0, int rpm = 200) // Changed from power to rpm
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
void autonomous()
{
	for (const auto &frame : frames)
	{
		left_motors.move_velocity(frame.left_input);
		right_motors.move_velocity(frame.right_input);
		lb.move_velocity(frame.lb_input);
		intake.move_velocity(frame.intake_power);
		solenoid.set_value(frame.solenoid_state);
		pros::delay(5);
	}
}
int quad_curve(int input, int max_rpm)
{
	float norm = input / 127.0f;
	int sign = (norm >= 0) ? 1 : -1;
	float curved = norm * norm * sign;
	return static_cast<int>(curved * max_rpm);
}
void opcontrol()
{
	static bool override_active = false;
	static bool solenoid_state = false;
	static bool doinker_state = false;
	static bool climb_state = false;
	
	while (true)
	{
		int power = quad_curve(master.get_analog(ANALOG_LEFT_Y), 600);
		int turn = quad_curve(master.get_analog(ANALOG_RIGHT_X), 600);
		int left_input = power + turn;
		int right_input = power - turn;

		if (master.get_digital_new_press(DIGITAL_B))
		{
			recording = !recording;
			if (!recording)
			{
				std::ofstream file("/usd/recorded.txt");
				for (const auto &frame : frames)
				{
					file << frame.left_input << " " << frame.right_input << " " << frame.lb_input << " " << frame.intake_power << " " << frame.solenoid_state << frame.doinker_state << std::endl;
				}
				file.close();
				frames.clear();
			}
		}
		if (recording)
		{
			record_frame frame;
			frame.left_input = left_input;
			frame.right_input = right_input;
			frame.lb_input = lb.get_actual_velocity();
			frame.intake_power = intake.get_actual_velocity();
			frame.solenoid_state = solenoid_state;
			frame.doinker_state = doinker_state;
			frames.push_back(frame);
		}
		if (master.get_digital(DIGITAL_RIGHT))
		{
			lb.move_velocity(200); // Green cartridge
		}
		else if (master.get_digital(DIGITAL_LEFT))
		{
			lb.move_velocity(-200); // Green cartridge
		}
		else
		{
			lb.move_velocity(0);
		}

		if (master.get_digital_new_press(DIGITAL_L2))
		{
			intake_running = true;
			intake_power = 200; // Green cartridge
		}
		if (master.get_digital_new_press(DIGITAL_R2))
		{
			intake_running = false;
			intake_power = 0;
		}
		if (master.get_digital(DIGITAL_L1))
		{
			intake_power = -200; // Green cartridge
		}
		else
		{
			intake_power = intake_running ? 200 : 0; // Green cartridge
		}

		if (master.get_digital_new_press(DIGITAL_R1))
		{
			solenoid_state = !solenoid_state;
			solenoid.set_value(solenoid_state);
		}
		if (master.get_digital_new_press(DIGITAL_A))
		{
			doinker_state = !doinker_state;
			doinker.set_value(doinker_state);
		}
		intake.move_velocity(intake_power);
		left_motors.move_velocity(left_input);  // Blue cartridge
		right_motors.move_velocity(right_input); // Blue cartridge
		pros::delay(5);
	}
}