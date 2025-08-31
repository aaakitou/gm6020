#ifndef INCLUDED_gm6020_H
#define INCLUDED_gm6020_H

#include "mbed.h"

class gm6020 {
    public:
        CANMessage gm6020_can_msg;
        gm6020(RawCAN &can,int motor_num);
        int gm6020_send(int* moter);
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        void can_read();
        float pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP=150,float KI=70, float KD=0);
        void deg_control(float* set_deg,int* motor,float* KP_GM6020,float* KI_GM6020,float* KD_GM6020);
        
    private:
        const float pi = 3.14159265;
        CANMessage _canMessage,_canMessage2,_msg;
        RawCAN &_can;
        int _motor_num,_motor_max;
        unsigned short _r;
        int _rotation;
        int _speed;
        int _torque;
        int _temperature;
        int _delta_deg;
        


};


#endif
