#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

enum class Colour
{
  RED,
  BLUE,
  NONE
};

extern pros::Optical colour_sensor;
extern pros::MotorGroup intake;
extern pros::Motor intake_conveyor;
extern pros::adi::DigitalOut mogo;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalIn limit_switch;
extern std::atomic<int32_t> intake_power;
extern ez::Drive chassis;

extern bool intake_running;
extern bool mogo_on;
extern bool doinker_on;
extern Colour team_colour;

void activate_doinker(bool value);
void activate_intake(int32_t voltage, uint32_t duration_ms = 0);
void activate_mogo(bool value);
bool is_intake_stalled(const pros::MotorGroup &motors, int32_t threshold = 60);
Colour detect_colour();
bool wait_for_ring(uint32_t timeout_ms = 1000);
void push_into_lb();
