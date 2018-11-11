#pragma once


namespace movement {

// motors can be in one of four states:
// - forward - turns in a way that moves robot forward
// - backwards - the other direction
// - stopped - holds motor one position (forces stopping)
// - neutral - no power (disabled), motor can move freely
//
// 3 first are set using set_direction(), last - by set_enabled(false)
enum MotorDirection {
    FORWARDS, BACKWARDS, STOPPED
};

namespace driver {

void initialise();
void set_enabled(bool enabled);
void set_direction(MotorDirection left, MotorDirection right);
void set_pulse(unsigned int left, unsigned int right);
unsigned int max_pulse();

} // namespace driver
} // namespace movement


