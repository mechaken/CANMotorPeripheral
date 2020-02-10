#ifndef CAN_MD_PERIPHERAL
#define CAN_MD_PERIPHERAL

#include "mbed.h"
#include "A3921.h"

class CANMDPeripheral : public A3921
{
public:
    CANMDPeripheral(CAN &can_obj, PinName sr, PinName pwmh, PinName pwml, PinName phase, PinName reset, PinName ledh, PinName ledl);
    ~CANMDPeripheral();

    void id(int value);
    int id();
    int connect();
    int decode_can_message(unsigned char *data);
    int decode_initial_value(unsigned char *data);
    void adjust();
    void release_time_dec();
    int has_recieved_initial_value();

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
    int _has_recieved_initial_value;

    int _switching_wait_count_ms;

    int decode_extention_headers(unsigned char *data, int bit_number);
    int pwm_up_down(float now_duty_cycle, float goal_duty_cycle);
    int get_particular_bit(unsigned char *data, int bit_number);
    float bfloat16_decode(unsigned char *data, int bit_number);
    int int_decode(unsigned char *data, int bit_number, int length);
};

#endif