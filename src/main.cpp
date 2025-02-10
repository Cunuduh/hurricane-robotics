#include "main.h"

const double LB_MIN_ANGLE = 0;
const double LB_MAX_ANGLE = 110.0;

pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(20);
pros::IMU imu(21);

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

double lb_presets[] = {5.0, 17.0, LB_MAX_ANGLE};
size_t lb_preset_index = 0;
bool moving_lb = false;

struct action_frame
{
	uint16_t timestamp_delta;
	int8_t left_input;
	int8_t right_input;
	uint16_t flags; // flags: lb_right, lb_left, intake_l1, intake_l2, intake_r2, solenoid_toggle, doinker_toggle, climb_toggle, lb_up, lb_down

	inline bool operator!=(const action_frame &other) const
	{
		return left_input != other.left_input || right_input != other.right_input || flags != other.flags;
	}
};

bool recording = false;
std::vector<action_frame> frames;

int32_t cube_curve(int32_t input, int32_t max_rpm)
{
	float norm = input / 127.0f;
	float curved = norm * norm * norm;
	return static_cast<int32_t>(curved * max_rpm);
}
double get_lb_angle()
{
	return lb_rotation.get_angle() / 100.0;
}
void initialize()
{
	pros::lcd::initialize();
	master.clear();
	solenoid.set_value(solenoid_on);
	doinker.set_value(doinker_on);
	lb_rotation.reset_position();
	imu.reset();
	lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::ifstream file("/usd/tournament.bin", std::ios::binary);
	frames.reserve(24000);
	if (file.is_open())
	{
		frames.clear();
		action_frame frame;
		while (file.read(reinterpret_cast<char *>(&frame), sizeof(action_frame)))
			frames.push_back(frame);

		file.close();
		pros::delay(50);
		master.print(0, 0, "%d frames", frames.size());
	}
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

		int32_t power = cube_curve(frame.left_input, 600);
		int32_t turn = cube_curve(frame.right_input, 600);
		left_motors.move_velocity(power + turn);
		right_motors.move_velocity(power - turn);

		if (frame.flags & 0b1)
		{
			if (lb_preset_index > 0)
				lb_preset_index--;
		}
		if (frame.flags & 0b10)
		{
			if (lb_preset_index < 2)
				lb_preset_index++;
		}
		if (frame.flags & 0b100000000)
		{
			if (0 <= lb_preset_index && lb_preset_index < 2)
			{
				lb_preset_index++;
				moving_lb = true;
			}
		}
		if (frame.flags & 0b1000000000)
		{
			if (0 <= lb_preset_index && lb_preset_index < 2)
			{
				lb_preset_index--;
				moving_lb = true;
			}
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
		if (frame.flags & 0b1000)
		{
			intake_running = true;
		}
		else if (frame.flags & 0b10000)
		{
			intake_running = false;
		}

		if (intake_running)
		{
			if (frame.flags & 0b100)
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
		intake.move_velocity(intake_power);

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
void save_frames_to_file(const char *filename)
{
	std::ofstream file(filename, std::ios::binary);
	if (file.is_open())
	{
		for (size_t i = 0; i < frames.size(); i++)
		{
			if (i % 25 == 0 || i == frames.size() - 1)
				master.print(0, 0, "%d/%d", i + 1, frames.size());
			file.write(reinterpret_cast<const char *>(&frames[i]), sizeof(action_frame));
			pros::delay(2);
		}
		file.close();
		pros::delay(50);
		master.clear();
		pros::delay(50);
		master.print(0, 0, "Saved!       ", frames.size());
		pros::delay(500);
	}
}
void opcontrol()
{
	uint32_t last_timestamp = pros::millis();
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

		if (master.get_digital_new_press(DIGITAL_B) && !pros::competition::is_connected())
		{
			recording = !recording;
			if (!recording)
			{
				master.print(0, 0, "Saving...    ");
				pros::delay(500);
				master.print(0, 0, "A=Tournament ");
				pros::delay(50);
				master.print(1, 0, "Y=Skills     ");
				pros::delay(50);
				master.print(2, 0, "B=Cancel     ");
				pros::delay(50);
				while (true)
				{
					if (master.get_digital_new_press(DIGITAL_B))
					{
						frames.clear();
						master.clear();
						break;
					}
					else if (master.get_digital_new_press(DIGITAL_A))
					{
						save_frames_to_file("/usd/tournament.bin");
						master.clear();
						break;
					}
					else if (master.get_digital_new_press(DIGITAL_Y))
					{
						save_frames_to_file("/usd/auto.bin");
						master.clear();
						break;
					}
					pros::delay(10);
				}
			}
			else
			{
				frames.clear();
				master.print(0, 0, "Recording    ");
			}
		}

		if (recording)
		{
			static uint32_t last_timestamp = pros::millis();
			static action_frame previous_frame{0, 0, 0, 0};
			action_frame frame;
			uint32_t current_time = pros::millis();
			frame.timestamp_delta = current_time - last_timestamp;
			frame.left_input = analog_left_y;
			frame.right_input = analog_right_x;
			frame.flags =
					(master.get_digital(DIGITAL_RIGHT) << 0) |
					(master.get_digital(DIGITAL_LEFT) << 1) |
					(master.get_digital(DIGITAL_L1) << 2) |
					(l2_press << 3) |
					(r2_press << 4) |
					(r1_press << 5) |
					(a_press << 6) |
					(x_press << 7) |
					(master.get_digital(DIGITAL_UP) << 8) |
					(master.get_digital(DIGITAL_DOWN) << 9);
			if (frame != previous_frame || frames.empty())
			{
				frames.push_back(frame);
				previous_frame = frame;
				last_timestamp = current_time;
			}
			pros::lcd::print(0, "%d,%d,%d", frame.left_input, frame.right_input, frame.flags);
		}
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
		if (x_press)
		{
			climb_on = !climb_on;
			climb.set_value(climb_on);
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