#ifndef MOTOR_DEFINE_HPP
#define MOTOR_DEFINE_HPP

// moving type
#define MoveAbsolutePostion 1
#define MoveRelativePosition 2
#define SearchHome 3
#define StopMotor 4
#define ResetMotor 5

#define MotorStop 1
#define MotorHoming 2

#define CHANNEL_1 1
#define CHANNEL_2 2
#define SET_MOTOR_CMD_ADDR_S32 0x2000  // 1~number of motor
#define SET_POS_ADDR_S32 0x2001        // 1~number of motor
#define SET_VEL_ADDR_S32 0x2002        // 1~number of motor
#define SE_ENC_COUNTER_ADDR_S32 0x2003 // 1~number of encoder
#define SET_USER_VAR_ADDR_S32 0x2005   // 1~number of integer variable
#define ACC_ADDR_S32 0x2006            // 1~number of motor
#define DEC_ADDR_S32 0x2007            // 1~number of motor
#define EMC_SHUTDOWN_ADDR_U8 0x200C
#define RELEASE_SHUTDOWN_ADDR_U8 0x200D
#define STOP_ALL_MODE_ADDR_U8 0x200E
#define BRAKE_OVERRIDE_ADDR_U8 0x2034
#define HOMING_VAR 0x2005
#define HOMING_CHANNEL_1 17
#define HOMING_CHANNEL_2 18
#define STOP_VAR 0x2005
#define STOP_CHANNEL_1 19
#define STOP_CHANNEL_2 20

#define XAMotorOverHeat 2604    // f1
#define XAMotorOverVolt 2605    // f2
#define XAMotorUnderVolt 2606   // f3
#define XAMotorShort 2607       // f4
#define XAMotorEstop 2608       // f5
#define XAMotorSensorFault 2609 // f6
#define XAMotorMosFail 2610     // f7
#define XAMotorDefConfig 2611   // f8
#define XAMotorSTOFault 2612    // f9

#define TYMotorOverHeat 2613    // f1
#define TYMotorOverVolt 2614    // f2
#define TYMotorUnderVolt 2615   // f3
#define TYMotorShort 2616       // f4
#define TYMotorEstop 2617       // f5
#define TYMotorSensorFault 2618 // f6
#define TYMotorMosFail 2619     // f7
#define TYMotorDefConfig 2620   // f8
#define TYMotorSTOFault 2621    // f9

#define ZMotorOverHeat 2622    // f1
#define ZMotorOverVolt 2623    // f2
#define ZMotorUnderVolt 2624   // f3
#define ZMotorShort 2625       // f4
#define ZMotorEstop 2626       // f5
#define ZMotorSensorFault 2627 // f6
#define ZMotorMosFail 2628     // f7
#define ZMotorDefConfig 2629   // f8
#define ZMotorSTOFault 2630    // f9
#define MotorError 2631

#endif // MOTOR_DEFINE_HPP
