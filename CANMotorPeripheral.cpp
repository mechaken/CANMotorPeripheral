#include "CANMotorPeripheral.h"
#include "mbed.h"
#include "bfloat16.h"

CANMotorPeripheral::CANMotorPeripheral(CAN &can_obj, PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset, PinName ledh, PinName ledl)
    : A3921(sr, pwmh, pwml, phase, reset), _can_p(NULL), _can(can_obj)
{
    _led = new BusOut(ledh, ledl);
    _rise_unit = convert_level(_rise_level);
    _fall_unit = convert_level(_fall_level);

    _time_out_count = 0;
}

CANMotorPeripheral::~CANMotorPeripheral()
{
    delete _led;
}

void CANMotorPeripheral::id(int value)
{
    if ((0 <= value) && (value <= 0x7FF))
        _id = value;
}

int CANMotorPeripheral::id()
{
    return _id;
}

int CANMotorPeripheral::decode_can_message(unsigned char *data)
{
    if ((data[0] & 0x80) == 0x00) {
        _duty_cycle = (float)(((data[0] & 0x7F) << 9) + (data[1] << 1) + ((data[2] & 0x80) >> 7)) / 65536.0f; // 2^16 = 65536

        _state = (State)((data[2] & 0x60) >> 5);

        _time_out_count = _release_time_ms;

        adjust();
        return 0;
    }

    return 1;
}

void CANMotorPeripheral::adjust()
{
    float current_duty_cycle = hal_duty_cycle();
    int current_state = hal_state();
    float difference = _duty_cycle - current_duty_cycle;

    float tmp_duty_cycle;
    int tmp_state;

    if (current_state != _state) {
        if (_fall_level == OFF) {
            tmp_duty_cycle = _duty_cycle;
            tmp_state = _state;
        } else if (current_duty_cycle <= _fall_unit) {
            tmp_duty_cycle = 0.0f;
            tmp_state = _state;
        } else {
            tmp_duty_cycle = current_duty_cycle - _fall_unit;
            tmp_state = current_state;
        }
    } else if (current_duty_cycle != _duty_cycle) {
        if (_state == Brake || _state == Free)
            return;

        tmp_state = _state;

        // 設定値の方が現在のデューティー比より大きいとき
        if (difference >= _rise_unit) {

            if (_rise_level == OFF) {
                tmp_duty_cycle = _duty_cycle;
            } else {
                tmp_duty_cycle = current_duty_cycle + _rise_unit;
            }
        }
        // 設定値の方が現在のデューティー比より小さいとき
        else if (difference <= -_fall_unit) {

            if (_fall_level == OFF) {
                tmp_duty_cycle = _duty_cycle;
            } else {
                tmp_duty_cycle = current_duty_cycle - _fall_unit;
            }
        }
        // だいたい一緒の時
        else {
            tmp_duty_cycle = _duty_cycle;
            tmp_state = _state;
        }
    } else {
        // デバッグがしやすいから、宣言時でなくて最後に代入している
        tmp_duty_cycle = _duty_cycle;
        tmp_state = _state;
    }

    // debug("%f,%d\r",tmp_duty_cycle, tmp_state);
    hal_set(tmp_duty_cycle, tmp_state);

    if (_time_out_count != 0) {
        _led->write(tmp_state);
    }
}

void CANMotorPeripheral::release_time_dec()
{
    static int led_blink_count = 0;
    if (_time_out_count == 0) {
        // debug("Release Motor\r");

        // hal_setしないのは急な停止をさせないため（adjust()でだんだんと止めていく）
        _state = Brake;
        _duty_cycle = 0.0f;

        if(led_blink_count-- == 0) {
            led_blink_count = 200; // 200ms周期で点滅
            if (_led->read() != 1) {
                _led->write(1);
            } else {
                _led->write(2);
            }
        }

        printf("release\n");
    } else {
        _time_out_count--;
        led_blink_count = 0;
    }
}

float CANMotorPeripheral::convert_level(int level)
{
    // switch (level)
    // {
    // case Low:
    //     return 0.01f;
    // case Middle:
    //     return 0.005f;
    // case High:
    //     return 0.001f;
    // default:
    //     return 0.0f;
    // }

    if (level == 0) {
        return 0.0f;
    }

    float value = .002f;
    for (int i = 0; i < level; i++) {
        value *= 0.7f;
    }

    return value;
}
