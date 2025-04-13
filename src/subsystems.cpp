#include "main.h"

pros::Optical colour_sensor(19);
pros::Rotation lb_rotation(11);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
pros::Motor intake_conveyor(8, pros::v5::MotorGears::green);
pros::Motor lb(10, pros::v5::MotorGears::green);
pros::adi::DigitalOut mogo('A');
pros::adi::DigitalOut doinker('B');
std::atomic<int32_t> intake_power{0};
ez::PID lb_pid{1.5, 0.0, 0.3};

bool intake_running = false;
bool mogo_on = false;
bool doinker_on = false;
bool colour_rejection_active = false;
uint32_t last_rejection_time = 0;
Colour team_colour = Colour::BLUE;
double get_lb_angle()
{
  return lb_rotation.get_position() / 100.0;
}

void activate_doinker(bool value)
{
  doinker_on = value;
  doinker.set_value(doinker_on);
}

void activate_intake(int rpm, int duration_ms)
{
  intake_power = rpm;
  intake.move_velocity(intake_power);
  if (duration_ms == 0)
    return;
  pros::delay(duration_ms);
  intake.move_velocity(0);
}

void activate_lb(int velocity, int duration_ms)
{
  lb.move_velocity(velocity);
  pros::delay(duration_ms);
  lb.move_velocity(0);
}

void move_lb(int velocity)
{
  lb.move(velocity);
}

void activate_mogo(bool value)
{
  mogo_on = value;
  mogo.set_value(mogo_on);
}

void set_lb_stage(LBStage stage)
{
  switch (stage)
  {
    case LBStage::START:
      lb_pid.target_set(0);
      break;
    case LBStage::PICKUP:
      lb_pid.target_set(32);
      break;
    case LBStage::REACH:
      lb_pid.target_set(60);
      break;
    case LBStage::SCORE:
      lb_pid.target_set(160);
      break;
    case LBStage::END:
      lb_pid.target_set(270);
      break;
  }
}

void lb_pid_wait()
{
  while (lb_pid.exit_condition(lb_rotation.get_angle() / 100.0) == ez::RUNNING)
  {
    pros::delay(10);
  }
}

bool is_intake_stalled(const pros::MotorGroup &motors, int threshold)
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

Colour detect_colour()
{
  double hue = colour_sensor.get_hue();
  if (colour_sensor.get_proximity() < 40)
  {
    return Colour::NONE;
  }
  if ((hue >= 0.0 && hue <= 50.0) || (hue >= 330.0 && hue <= 359.999))
  {
    return Colour::RED;
  }
  else if (hue >= 90.0 && hue <= 270.0)
  {
    return Colour::BLUE;
  }
  else
  {
    return Colour::NONE;
  }
}

bool wait_for_ring(int timeout_ms)
{
  if (std::abs(lb_pid.target_get() - 32) > 10)
    return false;
    
  uint32_t start_time = pros::millis();
  
  while (pros::millis() - start_time < timeout_ms)
  {
    if (is_intake_stalled(intake, 50))
  {
      pros::delay(100);
      return true;
    }
    
    pros::delay(10);
  }
  
  return false;
}
