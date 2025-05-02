#include "main.h"
#include "lady_brown.hpp"

pros::Motor lb(10, pros::v5::MotorGears::green);
pros::Rotation lb_rotation{11};

LadyBrownArm lady_brown(lb, lb_rotation);

LadyBrownArm::LadyBrownArm(pros::Motor& lb_motor, pros::Rotation& lb_rotation)
    : motor(lb_motor), rotation_sensor(lb_rotation), 
      pid(3, 0.0, 1.0), current_stage(LBStage::START)
{
  motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  pid.exit_condition_set(80, 1, 250, 3, 250, 250);
  pid.target_set(START_ANGLE);
}

double LadyBrownArm::get_angle()
{
  return rotation_sensor.get_position() / 100.0;
}

void LadyBrownArm::move(int32_t velocity)
{
  motor.move(velocity);
}

double LadyBrownArm::get_stage_angle(LBStage stage)
{
  switch (stage)
  {
    case LBStage::START:
      return START_ANGLE;
    case LBStage::PICKUP:
      return PICKUP_ANGLE;
    case LBStage::REACH:
      return REACH_ANGLE;
    case LBStage::SCORE:
      return SCORE_ANGLE;
    case LBStage::END:
      return END_ANGLE;
    default:
      return START_ANGLE;
  }
}

void LadyBrownArm::set_stage(LBStage stage)
{
  current_stage = stage;
  pid.target_set(get_stage_angle(stage));
}

LBStage LadyBrownArm::get_stage()
{
  return current_stage;
}

void LadyBrownArm::wait_until_settled()
{
  while (pid.exit_condition(get_angle()) == ez::RUNNING)
  {
    pros::delay(ez::util::DELAY_TIME);
  }
}

void LadyBrownArm::update()
{
  move(ez::util::clamp(pid.compute(get_angle()), 127.0, -127.0));
}
