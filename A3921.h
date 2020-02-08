#ifndef A3921_H
#define A3921_H

#include "mbed.h"
#include "Motor.h"

class A3921 : public Motor
{
public:
    A3921(PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset);

    void hal_reset();
    void reset();
    void recovery();

    void hal_set(float duty_cycle, int state);
    float hal_duty_cycle();
    int hal_state();
    void hal_pulse_period(float seconds);
    void hal_frequency(float hz);

private:
    DigitalOut _sr;
    PwmOut _pwmh;
    PwmOut _pwml;
    DigitalOut _phase;
    DigitalOut _reset;

    float _hal_duty_cycle;
    int _hal_state;
};

#endif