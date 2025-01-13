#include "main.h"

const double LB_GEAR_RATIO = 4.0;
const double LB_MIN_ANGLE = 15.0;
const double LB_MAX_ANGLE = 120.0;
double flip = 1.0;

pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
auto imu = std::make_shared<okapi::IMU>(7);

pros::Motor lb(10, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
pros::MotorGroup left_motors({-4, -5, 6}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({1, 2, -3}, pros::v5::MotorGears::blue);

pros::ADIDigitalOut solenoid('A');
pros::ADIDigitalOut doinker('B');
pros::ADIDigitalOut climb('C');

pros::Controller master(pros::E_CONTROLLER_MASTER);

int32_t intake_power = 0;
bool intake_running = false;
bool solenoid_on = false;
bool doinker_on = false;
bool climb_on = false;

struct action_frame
{
	uint16_t timestamp_delta;
	int8_t left_input;
	int8_t right_input;
	uint8_t flags; // flags: lb_right, lb_left, intake_l1, intake_l2, intake_r2, solenoid_toggle, doinker_toggle, climb_toggle
};

struct controller_state
{
	int32_t left_input = 0;
	int32_t right_input = 0;
	bool lb_right = false;
	bool lb_left = false;
	bool intake_l1 = false;
	bool intake_l2 = false;
	bool intake_r2 = false;
	bool solenoid_toggle = false;
	bool doinker_toggle = false;
	bool climb_toggle = false;

	bool operator!=(const controller_state &other) const
	{
		return left_input != other.left_input ||
					 right_input != other.right_input ||
					 lb_right != other.lb_right ||
					 lb_left != other.lb_left ||
					 intake_l1 != other.intake_l1 ||
					 intake_l2 != other.intake_l2 ||
					 intake_r2 != other.intake_r2 ||
					 solenoid_toggle != other.solenoid_toggle ||
					 doinker_toggle != other.doinker_toggle ||
					 climb_toggle != other.climb_toggle;
	}
};

controller_state prev_state;
int64_t last_record_time = 0;

bool recording = false;
std::vector<action_frame> frames;
void initialize()
{
	pros::lcd::initialize();
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake_power = 0;
	intake_running = false;
	std::ifstream file("/usd/tournament.bin", std::ios::binary);
	frames.reserve(24000);
	if (file.is_open())
	{
		frames.clear();
		action_frame frame;
		while (file.read(reinterpret_cast<char *>(&frame), sizeof(action_frame)))
			frames.push_back(frame);

		file.close();
		master.print(0, 0, "%d frames", frames.size());
	}
	solenoid.set_value(solenoid_on);
	doinker.set_value(doinker_on);
}
void disabled() {}
void competition_initialize() {}
void autonomous()
{
	if (frames.empty())
		return;

	uint32_t start_time = pros::millis();
	uint32_t current_time = start_time;

	for (const auto &frame : frames)
	{
		current_time += frame.timestamp_delta;
		while (pros::millis() < current_time)
			pros::delay(1);

		int32_t power = quad_curve(frame.left_input, 600);
		int32_t turn = quad_curve(frame.right_input, 600);
		left_motors.move_velocity(power + turn);
		right_motors.move_velocity(power - turn);

		lb.move_velocity((frame.flags & 0b1) ? 200 : (frame.flags & 0b10) ? -200
																																 : 0);

		if (frame.flags & 0b100)
			intake.move_velocity(-200);
		else if (frame.flags & 0b1000)
			intake.move_velocity(200);
		else if (frame.flags & 0b10000)
			intake.move_velocity(0);

		if (frame.flags & 0b100000)
		{
			solenoid_on = !solenoid_on;
			solenoid.set_value(solenoid_on);
		}
		if (frame.flags & 0b1000000)
		{
			doinker_on = !doinker_on;
			doinker.set_value(doinker_on);
		}
		if (frame.flags & 0b10000000)
		{
			climb_on = !climb_on;
			climb.set_value(climb_on);
		}
	}

	left_motors.move_velocity(0);
	right_motors.move_velocity(0);
	lb.move_velocity(0);
	intake.move_velocity(0);
}
int32_t quad_curve(int32_t input, int32_t max_rpm)
{
	float norm = input / 127.0f;
	int32_t sign = (norm >= 0) ? 1 : -1;
	float curved = norm * norm * sign;
	return static_cast<int32_t>(curved * max_rpm);
}
double get_lb_angle()
{
	return lb_rotation.get_angle() / 100.0 * LB_GEAR_RATIO;
}
void save_frames_to_file(const char *filename)
{
	std::ofstream file(filename, std::ios::binary);
	if (file.is_open())
	{
		for (size_t i = 0; i < frames.size(); i++)
		{
			master.print(0, 0, "Saving %d/%d", i + 1, frames.size());
			file.write(reinterpret_cast<const char *>(&frames[i]), sizeof(action_frame));
		}
		file.close();
		pros::delay(100);
		master.clear();
		master.print(0, 0, "Saved!", frames.size());
		pros::delay(500);
		master.clear();
	}
}
void opcontrol()
{
	uint32_t last_timestamp = pros::millis();
	while (true)
	{
		int32_t power = quad_curve(master.get_analog(ANALOG_LEFT_Y), 600);
		int32_t turn = quad_curve(master.get_analog(ANALOG_RIGHT_X), 600);
		int32_t left_input = power + turn;
		int32_t right_input = power - turn;

		if (master.get_digital_new_press(DIGITAL_Y) && !pros::competition::is_connected())
			autonomous();

		if (master.get_digital_new_press(DIGITAL_B) && !pros::competition::is_connected())
		{
			recording = !recording;
			if (!recording)
			{
				master.clear();
				master.print(0, 0, "Saving...");
				pros::delay(500);
				master.clear();
				master.print(0, 0, "A=Tournament");
				master.print(1, 0, "Y=Skills");
				master.print(2, 0, "B=Cancel");
				while (true)
				{
					if (master.get_digital_new_press(DIGITAL_B))
					{
						frames.clear();
						break;
					}
					else if (master.get_digital_new_press(DIGITAL_A))
					{
						save_frames_to_file("/usd/tournament.bin");
						break;
					}
					else if (master.get_digital_new_press(DIGITAL_Y))
					{
						save_frames_to_file("/usd/auto.bin");
						break;
					}
					pros::delay(10);
				}
				master.clear();
			}
			else
			{
				frames.clear();
				master.clear();
				master.print(0, 0, "Recording");
			}
		}
		if (recording)
		{
			uint32_t current_time = pros::millis();
			controller_state current_state;
			current_state.left_input = master.get_analog(ANALOG_LEFT_Y);
			current_state.right_input = master.get_analog(ANALOG_RIGHT_X);
			current_state.lb_right = master.get_digital(DIGITAL_RIGHT);
			current_state.lb_left = master.get_digital(DIGITAL_LEFT);
			current_state.intake_l1 = master.get_digital(DIGITAL_L1);
			current_state.intake_l2 = master.get_digital_new_press(DIGITAL_L2);
			current_state.intake_r2 = master.get_digital_new_press(DIGITAL_R2);
			current_state.solenoid_toggle = master.get_digital_new_press(DIGITAL_R1);
			current_state.doinker_toggle = master.get_digital_new_press(DIGITAL_A);

			if ((current_state != prev_state) || frames.empty())
			{
				action_frame frame;
				frame.timestamp_delta = current_time - last_timestamp;
				frame.left_input = current_state.left_input;
				frame.right_input = current_state.right_input;

				frame.flags =
						(current_state.lb_right << 0) |
						(current_state.lb_left << 1) |
						(current_state.intake_l1 << 2) |
						(current_state.intake_l2 << 3) |
						(current_state.intake_r2 << 4) |
						(current_state.solenoid_toggle << 5) |
						(current_state.doinker_toggle << 6) |
						(current_state.climb_toggle << 7);

				frames.push_back(frame);
				prev_state = current_state;
			}
			last_timestamp = current_time;
		}
		if (master.get_digital(DIGITAL_RIGHT))
		{
			double current_angle = get_lb_angle();
			if (current_angle < LB_MAX_ANGLE)
				lb.move_velocity(200);
			else
				lb.move_velocity(0);
		}
		else if (master.get_digital(DIGITAL_LEFT))
		{
			double current_angle = get_lb_angle();
			if (current_angle > LB_MIN_ANGLE)
				lb.move_velocity(-200);
			else
				lb.move_velocity(0);
		}
		else
			lb.move_velocity(0);

		if (master.get_digital_new_press(DIGITAL_L2))
		{
			intake_running = true;
			intake_power = 200;
		}
		if (master.get_digital_new_press(DIGITAL_R2))
		{
			intake_running = false;
			intake_power = 0;
		}
		if (master.get_digital(DIGITAL_L1))
			intake_power = -200;
		else
			intake_power = intake_running ? 200 : 0;

		if (master.get_digital_new_press(DIGITAL_R1))
		{
			solenoid_on = !solenoid_on;
			solenoid.set_value(solenoid_on);
		}
		if (master.get_digital_new_press(DIGITAL_A))
		{
			doinker_on = !doinker_on;
			doinker.set_value(doinker_on);
		}
		if (master.get_digital_new_press(DIGITAL_X))
		{
			climb_on = !climb_on;
			climb.set_value(climb_on);
		}
		intake.move_velocity(intake_power);
		left_motors.move_velocity(left_input);
		right_motors.move_velocity(right_input);
		pros::delay(5);
	}
}