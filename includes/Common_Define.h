/*  Msg Type based VESC & PCAN */
#define PCAN_MESSAGE_STANDARD 0x00
#define PCAN_MESSAGE_RTR 0x01
#define PCAN_MESSAGE_EXTENDED 0x02
#define PCAN_MESSAGE_FD 0x03
#define PCAN_MESSAGE_BRS 0x04
#define PCAN_MESSAGE_ESI 0x05
#define PCAN_MESSAGE_ERRFRAME 0x06
#define PCAN_MESSAGE_STATUS 0x07

/*  COMM PACKET ID */
#define COMM_FW_VERSION 0
#define COMM_JUMP_TO_BOOTLOADER 1
#define COMM_ERASE_NEW_APP 2
#define COMM_WRITE_NEW_APP_DATA 3
#define COMM_GET_VALUES 4
#define COMM_SET_DUTY 5
#define COMM_SET_CURRENT 6
#define COMM_SET_CURRENT_BRAKE 7
#define COMM_SET_RPM 8
#define COMM_SET_POS 9
#define COMM_SET_HANDBRAKE 10
#define COMM_SET_DETECT 11 
#define COMM_SET_SERVO_POS 12
#define COMM_SET_MCCONF 13 
#define COMM_GET_MCCONF 14
#define COMM_GET_MCCONF_DEFAULT 15
#define COMM_SET_APPCONF 16
#define COMM_GET_APPCONF 17
#define COMM_GET_APPCONF_DEFAULT 18
#define COMM_SAMPLE_PRINT 19
#define COMM_TERMINAL_CMD 20
#define COMM_PRINT 21
#define COMM_ROTOR_POSITION 22
#define COMM_EXPERIMENT_SAMPLE 23
#define COMM_DETECT_MOTOR_PARAM 24
#define COMM_DETECT_MOTOR_R_L 25
#define COMM_DETECT_MOTOR_FLUX_LINKAGE 26
#define COMM_DETECT_ENCODER 27
#define COMM_DETECT_HALL_FOC 28
#define COMM_REBOOT 29
#define COMM_ALIVE 30
#define COMM_FORWARD_CAN 34
#define COMM_CUSTOM_APP_DATA 36
#define COMM_PING_CAN 62
#define COMM_GET_IMU_DATA 65

/*  CAN PACKET ID */
#define CAN_PACKET_SET_DUTY 0
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM 3
#define CAN_PACKET_SET_POS 4
#define CAN_PACKET_FILL_RX_BUFFER 5
#define CAN_PACKET_FILL_RX_BUFFER_LONG 6
#define CAN_PACKET_PROCESS_RX_BUFFER 7
#define CAN_PACKET_PROCESS_SHORT_BUFFER 8
#define CAN_PACKET_STATUS 9
#define CAN_PACKET_SET_CURRENT_REL 10
#define CAN_PACKET_SET_CURRENT_BRAKE_REL 11
#define CAN_PACKET_SET_CURRENT_HANDBRAKE 12
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REL 13
#define CAN_PACKET_STATUS_2 14
#define CAN_PACKET_STATUS_3 15
#define CAN_PACKET_STATUS_4 16
#define CAN_PACKET_PING 17
#define CAN_PACKET_PONG 18
#define CAN_PACKET_DETECT_APPLY_ALL_FOC 19
#define CAN_PACKET_DETECT_APPLY_ALL_FOC_RES 20
#define CAN_PACKET_CONF_CURRENT_LIMITS 21
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS 22
#define CAN_PACKET_CONF_CURRENT_LIMITS_IN 23
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN 24
#define CAN_PACKET_CONF_FOC_ERPMS 25
#define CAN_PACKET_CONF_STORE_FOC_ERPMS 26
#define CAN_PACKET_STATUS_5 27
#define CAN_PACKET_POLL_TS5700N8501_STATUS 28
#define CAN_PACKET_CONF_BATTERY_CUT 29
#define CAN_PACKET_CONF_STORE_BATTERY_CUT 30
#define CAN_PACKET_SHUTDOWN 31

/*  OPENRBOT & DRCL HOST Type */

#define UNKNOWN 0
#define ARDUINO_MEGA 1
#define ARDUINO_DUE 2
#define ARDUINO_TEENSY_32 3
#define ARDUINO_TEENSY_36 4
#define USB 5
#define CAN_DIRECT_MSG 6
#define ETHERCAT 7

/*  OPENRBOT & DRCL COMM PACKET ID */
#define COMM_SET_RELEASE 100
#define COMM_SET_DPS 101
#define COMM_SET_DPS_VMAX 102
#define COMM_SET_DPS_AMAX 103
#define COMM_SET_SERVO 104
#define COMM_SET_TRAJ 105


int status_negative_value_process_2_byte (int raw_value)
{
    int value;

    if (raw_value & 0x8000)
    {
        value = -((~raw_value & 0xffff) + 1);
    }
    else
    {
        value = raw_value;
    }

    return value;

}

int status_negative_value_process_3_byte (int raw_value)
{
    int value;

    if (raw_value & 0x800000)
    {
        value = -((~raw_value & 0xffffff) + 1);
    }
    else
    {
        value = raw_value;
    }

    return value;

}

int status_negative_value_process_4_byte (int raw_value)
{
    int value;

    if (raw_value & 0x80000000)
    {
        value = -((~raw_value & 0xffffffff) + 1);
    }
    else
    {
        value = raw_value;
    }

    return value;

}