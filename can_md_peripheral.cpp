#include "can_md_peripheral.h"
#include "mbed.h"
#include "bfloat16.h"

CANMDPeripheral::CANMDPeripheral(CAN &can_obj, PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset, PinName ledh, PinName ledl)
    : A3921(sr, pwmh, pwml, phase, reset), _can_p(NULL), _can(can_obj)
{
    _led = new BusOut(ledh, ledl);

    state(Brake);

    _duty_cycle = 0.0f;

    _release_time_ms = 100;
}

CANMDPeripheral::~CANMDPeripheral()
{
    delete _led;
}

void CANMDPeripheral::id(int value)
{
    if ((0 <= value) && (value <= 0x7FF))
        _id = value;
}

int CANMDPeripheral::id()
{
    return _id;
}

int CANMDPeripheral::connect()
{
    // IDに応じた待機時間を設ける（他のMDと競合しないようにするため）

    _msg.id = _id + 1;
    _msg.len = 1;
    _msg.data[0] = 0;

    _has_recieved_initial_value = false;

    int value = _can.write(_msg); // 初期設定値の送信を要求

    // 初期設定の通信が終わるまでLEDをチカチカさせる
    if (_led->read() != 1) {
        _led->write(1);
    } else {
        _led->write(3);
    }

    return value;
}

int CANMDPeripheral::decode_can_message(unsigned char *data)
{
    if ((data[0] & 0x80) == 0x80) {
        decode_initial_value(data);

        return 1;
    }

    _duty_cycle = (float)((data[0] << 7) + (data[1] << 1) + ((data[2] & 0x80) >> 7)) / 65536.0f; // 2^16 = 65536

    _state = (State)((data[2] & 0x60) >> 5);

    _time_out_count = _release_time_ms;

    adjust();

    // debug("d %0.3f, s %d\n", _duty_cycle,_state);

    return 0;
}

int CANMDPeripheral::decode_initial_value(unsigned char *data)
{
    _has_recieved_initial_value = true;

    if ((data[0] & 0x40) == 0x00) {
        _msg.data[0] = 1;
        _can.write(_msg); // ACKを送信

        decode_extention_headers(data, 2);
    }

    return 0;
}

int CANMDPeripheral::decode_extention_headers(unsigned char *data, int bit_number /* 0 ~ 63 (実質2 ~ 63) */)
{
    // Extention Headers
    // 00   -> Nothing
    // 010  -> pulse_period
    // 011  -> S/M only or LAP only (default -> S/M and LAP)
    // 10   -> DutyCycle Change Level
    // 110  -> release_time
    // 111  -> reserved for future use

    if (get_particular_bit(data, bit_number++) == 0) {
        if (get_particular_bit(data, bit_number++) == 0) {
            // 00
            return 0;
        } else {
            if (get_particular_bit(data, bit_number++) == 0) {
                //010
                int pulse_period = bfloat16_decode(data, bit_number);
                hal_pulse_period(pulse_period);

                bit_number += 16;
            } else {
                // 011
                _control = (Control)get_particular_bit(data, bit_number++);
            }
        }
    } else {
        if (get_particular_bit(data, bit_number++) == 0) {
            // 10
            _rise_level = (DutyCycleChangeLevel)int_decode(data, bit_number, 3);
            bit_number += 3;

            _fall_level = (DutyCycleChangeLevel)int_decode(data, bit_number, 3);
            bit_number += 3;
        } else {
            // 110
            _release_time_ms = bfloat16_decode(data, bit_number) * 1000;
            bit_number += 16;
        }
    }

    decode_extention_headers(data, bit_number);

    return 0;
}

int CANMDPeripheral::get_particular_bit(unsigned char *data, int bit_number)
{
    int subscript = bit_number / 8;
    int bit_num_of_subscript = bit_number % 8;

    if ((data[subscript] >> bit_num_of_subscript) & 0x01)
        return 1;
    else
        return 0;
}

void CANMDPeripheral::adjust()
{
    if (_has_recieved_initial_value == false)
        return;

    int tmp_duty_cycle, tmp_state;

    float current_duty_cycle = hal_duty_cycle();
    int current_state = hal_state();
    float difference = _duty_cycle - current_duty_cycle;

    if (current_state != _state) {
        // @TODO rise と fallの依存関係の解決
        if (_fall_level == OFF) {
            tmp_duty_cycle = _duty_cycle;
            tmp_state = _state;
        }
        if (current_duty_cycle <= _fall_unit) {
            tmp_duty_cycle = 0.0f;
            tmp_state = _state;
        } else if (current_duty_cycle > _fall_unit) {
            tmp_duty_cycle = current_duty_cycle - _fall_unit;
            tmp_state = current_state;
        }
    } else if (current_duty_cycle != _duty_cycle) {
        if (current_state == Brake || current_state == Free)
            return;

        // 設定値の方が現在のデューティー比より大きいとき
        if (difference >= _rise_unit) {

            if (_rise_level == OFF) {
                tmp_duty_cycle = _duty_cycle;
            } else {
                tmp_duty_cycle = current_duty_cycle + _rise_unit;
            }

            // 設定値の方が現在のデューティー比より小さいとき
        } else if (difference <= -_fall_unit) {

            if (_fall_level == OFF) {
                tmp_duty_cycle = _duty_cycle;
            } else {
                tmp_duty_cycle = current_duty_cycle + _rise_unit;
            }

            // だいたい一緒の時
        } else {
            tmp_duty_cycle = _duty_cycle;
        }
    } else {
        // デバッグがしやすいから、宣言時でなくて最後に代入している
        tmp_duty_cycle = _duty_cycle;
        tmp_state = _state;
    }

    hal_set(tmp_duty_cycle, tmp_state);
    _led->write(tmp_state);
}

void CANMDPeripheral::release_time_dec()
{
    if (_time_out_count == 0) {
        state(Brake);
    } else {
        _time_out_count--;
    }
}

int CANMDPeripheral::has_recieved_initial_value()
{
    return _has_recieved_initial_value;
}

float CANMDPeripheral::convert_level(int level)
{
    switch (level) {
        case Low:
            return 0.01f;
        case Middle:
            return 0.005f;
        case High:
            return 0.001f;
        default:
            return 0.0f;
    }
}

float CANMDPeripheral::bfloat16_decode(unsigned char *data, int bit_number)
{
    uint16_t bit_stream;
//    int subscript = bit_number / 8;
//    int bit_num_of_subscript = bit_number % 8;

    for (int i = 0; i < 16; i++) {
        bit_stream = (bit_stream << 1) | get_particular_bit(data, bit_number++);
    }

    return bfloat16::bfloat16_to_float32(bit_stream);
}
int CANMDPeripheral::int_decode(unsigned char *data, int bit_number, int length)
{
    uint32_t bit_stream;
//    int subscript = bit_number / 8;
//    int bit_num_of_subscript = bit_number % 8;

    for (int i = 0; i < length; i++)
    {
        bit_stream = (bit_stream << 1) | get_particular_bit(data, bit_number++);
    }

    return bit_stream;
}