#ifndef CAN_MOTOR_PERIPHERAL
#define CAN_MOTOR_PERIPHERAL

#include "mbed.h"
#include "A3921.h"

class CANMotorPeripheral : public A3921
{
public:
    CANMotorPeripheral(CAN &can_obj, PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset, PinName ledh, PinName ledl);
    ~CANMotorPeripheral();

    void id(int value);
    int id();
    int decode_can_message(unsigned char *data);
    void adjust();
    void release_time_dec();

protected:
    float convert_level(int level);

private:
    BusOut *_led;
    CAN *_can_p;
    CAN &_can;
    CANMessage _msg;

    int _id;
    float _rise_unit;
    float _fall_unit;
    int _time_out_count;

    int _switching_wait_count_ms;

    int decode_extention_headers(unsigned char *data, int bit_number);
    int pwm_up_down(float now_duty_cycle, float goal_duty_cycle);
};

#endif
