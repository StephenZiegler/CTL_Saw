//
#ifndef VescCAN_h
#define VescCAN_h


#include "definitions.h"                // SYS function prototypes
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "math.h"

 void  VescCan(void);
 void  VescCanFeedBelt(void);
 bool Button1Trigger();
 bool StartBtnTrigger();
 bool StopBtnTrigger();
 //class MyClass {       // The class
 // public:             // Access specifier
 //   int myNum;        // Attribute (int variable)
//    string myString;  // Attribute (string variable)
//};
 
 
 enum Axis {
    traverse = 1,
    saw = 2,
    belt_bot = 3,
    belt_top = 4
    };
 
 
//class ODriveCAN {
//public:
 
 typedef union {
     int8_t str[2]; 
     int16_t i16;
 }msg_int16;
 
 typedef union {
     int32_t i32;
     int8_t str[4];
 }msg_int32;
 
 typedef struct {
        msg_int16 DutyCycle;
        msg_int16 TotCurr;
        msg_int32 RPM;
        msg_int32 AmpHrsCharged;
        msg_int32 AmpHrs;
        msg_int32 WattHrsCharged;
        msg_int32 WattHrs;
        msg_int16 PidPos;
        msg_int16 TotInputCurr;
        msg_int16 MotorTemp;
        msg_int16 FetTemp;
        msg_int16 InputVolts;
        msg_int32 Tach;
    } VESC_StatMsg;

    typedef struct {
        VESC_StatMsg msg;
        float   DutyCycle;
        float   TotCurr;
        int32_t RPM;
        float   AmpHrsCharged;
        float   AmpHrs;
        float   WattHrsCharged;
        float   WattHrs;
        float   PidPos;
        float   TotInputCurr;
        float   MotorTemp;
        float   FetTemp;
        float   InputVolts;
        int32_t Tach;
    }VESC_Data;
    
    

//    typedef struct {
//        float v_in;
//        float temp_mos1;
//        float temp_mos2;
//        float temp_mos3;
//        float temp_mos4;
//        float temp_mos5;
//        float temp_mos6;
//        float temp_pcb;
//        float current_motor;
//        float current_in;
//        float rpm;
//        float duty_now;
//        float amp_hours;
//        float amp_hours_charged;
//        float watt_hours;
//        float watt_hours_charged;
//        int tachometer;
//        int tachometer_abs;
//        uint8_t fault_code;
//    } VESC_StatMsg;

    enum VCommandId_t {
      //  CMD_ID_CANOPEN_NMT_MESSAGE = 0x000,
        CMD_ID_SET_DUTY = 0,  
        CMD_ID_SET_CURRENT = 1,
        CMD_ID_SET_CURRENT_BRAKE = 2,
        CMD_ID_SET_RPM = 3,
        CMD_ID_SET_POS = 4,                                                                                                         
        CMD_ID_FILL_RX_BUFFER = 5,
        CMD_ID_FILL_RX_BUFFER_LONG = 6,
        CMD_ID_PROCESS_RX_BUFFER = 7,
        CMD_ID_PROCESS_SHORT_BUFFER = 8,
        CMD_ID_STATUS = 9,
        CMD_ID_SET_CURRENT_REL = 10,
        CMD_ID_SET_CURRENT_BRAKE_REL = 11,
        CMD_ID_SET_CURRENT_HANDBRAKE = 12,
        CMD_ID_SET_CURRENT_HANDBRAKE_REL = 13,
        CMD_ID_STATUS_2 = 14,
        CMD_ID_STATUS_3 = 15,
        CMD_ID_STATUS_4 = 16,
        CMD_ID_PING = 17,
        CMD_ID_PONG = 18,
        CMD_ID_DETECT_APPLY_ALL_FOC = 19,
        CMD_ID_DETECT_APPLY_ALL_FOC_RES = 20,
        CMD_ID_CONF_CURRENT_LIMITS = 21,
        CMD_ID_CONF_STORE_CURRENT_LIMITS = 22,
        CMD_ID_CONF_CURRENT_LIMITS_IN = 23,
        CMD_ID_CONF_STORE_CURRENT_LIMITS_IN = 24,
        CMD_ID_CONF_FOC_ERPMS = 25,
        CMD_ID_CONF_STORE_FOC_ERPMS = 26,
        CMD_ID_STATUS_5 = 27
    };
    
    typedef union {
        struct {
            unsigned ValidMsgs       :1;
            unsigned bool1           :1;
            unsigned bool2           :1;
            unsigned bool3           :1;
            unsigned bool4           :1;
            unsigned bool5           :1;
            unsigned bool6           :1;
            unsigned bool7           :1;
            unsigned bool8           :1;
            unsigned bool9           :1;
            unsigned bool10          :1;
            unsigned bool11          :1;
            unsigned bool12          :1;
            unsigned bool13          :1;
            unsigned bool14          :1;
            unsigned bool15          :1;
            unsigned bool16          :1;
            unsigned bool17          :1;
            unsigned bool18          :1;
            unsigned bool19          :1;
            unsigned bool20          :1;
            unsigned bool21          :1;
            unsigned bool22          :1;
            unsigned bool23          :1;
            unsigned bool24          :1;
            unsigned bool25          :1;
            unsigned bool26          :1;
            unsigned bool27          :1;
            unsigned bool28          :1;
            unsigned bool29          :1;
            unsigned bool30          :1;
            unsigned bool31          :1;            
        }bit;
        int32_t LNG;
    }VESC_FLAGS;
    
    VESC_FLAGS VescFlags;
    //VESC_FLAGS Axis;
    
    VESC_Data VescData;
    VESC_Data Traverse;
    VESC_Data Saw;
    VESC_Data FeedBelt;
    //VESC_Data Axis;
    
 
    float RampUp(float RampValue,float EndValue);
	void sendVescMessage(int axis_id,int cmd_id,int length, uint8_t *data,uint8_t fifoNum, bool remote_transmission_request);
    void receiveVescMessage(int unit_id);
    void SetDuty(int unit_id,float Duty); // set duty 00
    void SET_CURRENT(int unit_id,float current);// set current 1
    void SET_CURRENT_BRAKE(int unit_id,float brake_current);// set brake current 1
    void SetPos(int unit_id, int32_t Pos);
    void SetPos2(int unit_id, int32_t Pos);
    
    void SetRPM(int unit_id,int32_t RPM); // set rpm 3
    int32_t GetRPM(int axis_id);  // rpm part of status 9
    float GetCurrent(int axis_id);
    float GetDuty(int axis_id);
    int64_t GetStatus(int axis_id);
    void SetCurrentLimits(int axis_id, float AmpLimitPlus, float AmpLimitMinus);  // current limits 21
    
#endif