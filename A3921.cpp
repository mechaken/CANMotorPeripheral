#include "A3921.h"

A3921::A3921(PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset)
    : Motor(), _sr(sr), _pwmh(pwmh), _pwml(pwml), _phase(phase), _reset(reset)
{
    _reset.write(1);
}

void A3921::hal_reset()
{
    // _reset = 0;
    // 急に止まってほしくないからhal_set()はしない
    _pwmh.period(default_pulse_period);
    _pwml.period(default_pulse_period);
}

void A3921::recovery()
{
    _reset = 1; // A3921から5V持ってきている設計だと使えない（と思う）
}

void A3921::hal_set(float duty_cycle, int state)
{
    _hal_state = state;
    //    switch (state)
    //    {
    //    case Free:
    //        _pwmh = 0.0f;
    //        _pwml = 0.0f;
    //
    //        _hal_duty_cycle = 0.0f;
    //        break;
    //    case CW:
    //        _phase = 1; // a to b
    //        _pwmh = duty_cycle;
    //        _pwml = 1.0f;
    //
    //        _hal_duty_cycle = duty_cycle;
    //        break;
    //    case CCW:
    //        _phase = 0; // b to a
    //        _pwmh = duty_cycle;
    //        _pwml = 1.0f;
    //
    //        _hal_duty_cycle = duty_cycle;
    //
    //        break;
    //    case Brake:
    //        _pwmh = 0.0f;
    //        _pwml = 1.0f;
    //
    //        _hal_duty_cycle = 0.0f;
    //
    //        break;
    //    default:
    //        _hal_duty_cycle = -1.0f;
    //        _hal_state = -1;
    //        break;
    //    }

    switch (_state) {
        case Free:
            _pwmh = 0.0f;
            _pwml = 0.0f;

            _hal_duty_cycle = 0.0f;
            break;
        case CW:
            _phase = 1; // a to b
            _pwmh = _duty_cycle;
            _pwml = _duty_cycle;

            _hal_duty_cycle = _duty_cycle;
            break;
        case CCW:
            _phase = 0; // b to a
            _pwmh = _duty_cycle;
            _pwml = _duty_cycle;

            _hal_duty_cycle = _duty_cycle;
            break;
        case Brake:
            _pwmh = 0.0f;
            _pwml = 0.0f;

            _hal_duty_cycle = 0.0f;
            break;
        default:
            _hal_duty_cycle = -1.0f;
            _hal_state = -1;
            break;
    }
    //     }
}

float A3921::hal_duty_cycle()
{
    return _hal_duty_cycle;
}

int A3921::hal_state()
{
    return _hal_state;
}

void A3921::hal_pulse_period(float seconds)
{
    if ((0 < seconds) && (seconds <= max_pulse_period)) {
        _pwmh.period(seconds);
        _pwml.period(seconds);
    }
}

void A3921::hal_frequency(float hz)
{
    hal_pulse_period(1.0f / hz);
}
