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

enum class Colour
{
  RED,
  BLUE,
  NONE
};

extern Drive chassis;
extern ez::PID lb_pid;
extern pros::MotorGroup intake;
extern pros::adi::DigitalOut mogo;
extern pros::adi::DigitalOut doinker;
extern pros::Motor lb;
extern pros::Rotation lb_rotation;
extern pros::Optical colour_sensor;
extern Colour team_colour;
extern std::atomic<int32_t> intake_power;
extern bool intake_running;
extern bool mogo_on;
extern bool doinker_on;
extern bool colour_rejection_active;

double get_lb_angle();
void activate_doinker(bool value);
void activate_intake(int32_t rpm, uint32_t duration_ms = 0);
void activate_lb(int32_t velocity, uint32_t duration_ms = 0);
void move_lb(int32_t velocity);
void activate_mogo(bool value);
void set_lb_stage(LBStage stage);
void lb_pid_wait();
bool is_intake_stalled(const pros::MotorGroup &motors, int32_t threshold = 60);
void attempt_unjam();
Colour detect_colour();
bool wait_for_ring(uint32_t timeout_ms = 2000);
