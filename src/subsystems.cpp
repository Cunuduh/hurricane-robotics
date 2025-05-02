#include "main.h"
#include "subsystems.hpp"
#include "lady_brown.hpp"

pros::Optical colour_sensor(19);
pros::MotorGroup intake({8, -9}, pros::v5::MotorGears::green);
pros::Motor intake_conveyor(8, pros::v5::MotorGears::green);
pros::adi::DigitalOut mogo('A');
pros::adi::DigitalOut doinker('B');
pros::adi::DigitalIn limit_switch('H');
std::atomic<int32_t> intake_power{0};

bool intake_running = false;
bool mogo_on = false;
bool doinker_on = false;
Colour team_colour = Colour::BLUE;

void activate_doinker(bool value)
{
  doinker_on = value;
  doinker.set_value(doinker_on);
}

void activate_intake(int32_t voltage, uint32_t duration_ms)
{
  intake_power = voltage;
  intake.move(intake_power);
  if (duration_ms == 0)
    return;
  pros::delay(duration_ms);
  intake.move(0);
}

void activate_mogo(bool value)
{
  mogo_on = value;
  mogo.set_value(mogo_on);
}

bool is_intake_stalled(const pros::MotorGroup &motors, int32_t threshold)
{
  return std::abs(motors.get_actual_velocity_all()[0]) < 10 &&
         std::abs(motors.get_target_velocity_all()[0]) > threshold;
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

bool wait_for_ring(uint32_t timeout_ms)
{
  if (std::abs(lady_brown.get_angle() - lady_brown.get_stage_angle(LBStage::PICKUP)) > 25.0)
    return false;
  auto mark = pros::millis();
  while (!is_intake_stalled(intake, 50) && pros::millis() - mark < timeout_ms)
    pros::delay(2);
  return (pros::millis() - mark < timeout_ms);
}

void push_into_lb()
{
  if (!wait_for_ring()) return;
  pros::delay(50);
  intake.move(0);
  pros::delay(10);
  for (int i = 0; i < 4; i++)
  {
    intake_conveyor.move(127);
    pros::delay(250);
    intake_conveyor.move(0);
    pros::delay(50);
  }
  intake_conveyor.move(intake_power);
}
