#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

enum class LBStage
{
  START,
  PICKUP,
  REACH,
  SCORE,
  END,
};

extern pros::Motor lb;
extern pros::Rotation lb_rotation;

class LadyBrownArm {
private:
  pros::Motor& motor;
  pros::Rotation& rotation_sensor;
  ez::PID pid;
  LBStage current_stage;

  const double START_ANGLE = 0.0;
  const double PICKUP_ANGLE = 29.0;
  const double REACH_ANGLE = 75.0;
  const double SCORE_ANGLE = 150.0;
  const double END_ANGLE = 270.0;

public:
  LadyBrownArm(pros::Motor& lb_motor, pros::Rotation& lb_rotation);
  
  void move(int32_t velocity);
  double get_angle();
  void set_stage(LBStage stage);
  LBStage get_stage();
  double get_stage_angle(LBStage stage);
  
  void wait_until_settled();
  
  void update();
};

extern LadyBrownArm lady_brown;
