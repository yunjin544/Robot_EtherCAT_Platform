#include "VESCular_Command.h"
#include "array_sum.h"
#include "VESCular_crc.h"
#include "Common_Define.h"
#include <iostream>

using namespace std;

#define BYTE_NUM 32


struct Ecat_raw
{
    int hostype;
    int command;
    int vesc_id;
    float set_value;
};

class VESC_VALUES{
    public:
        float temp_fet = 0;
        float temp_motor = 0;
        float motor_current = 0;
        float input_current = 0;
        float id_current = 0;
        float iq_current = 0;
        float duty = 0;
        float erpm = 0;
        float volt_input = 0;
        float amp_hours = 0;
        float amp_hours_charged = 0;
        float watt_hours = 0;
        float watt_hours_charged = 0;
        float tacho = 0;
        float tacho_abs = 0;
        float fault = 0;
        float pid_pos_now = 0;
        float controller_id = 0;
        float temp_mos1 = 0;
        float temp_mos2 = 0;
        float temp_mos3 = 0;
        float vd_volt = 0;
        float vq_volt = 0;

};

class VESC_PACKET{
    private:

    public:
    int* packet_encoding( int comm , Ecat_raw data );
    int ethercat_send_cmd(int slave, int cmd , float value );
    int get_bytes(int data[], int length , float div);
    int crc_check(int dataframe[], int len, int com_crch , int com_crcl);
   
};

int* VESC_PACKET::packet_encoding( int comm , Ecat_raw data ){
    int *packet_data = new int[BYTE_NUM];
    int start_frame=2;  //start_frame = 2
    int cmd_size = 0;
    int cmd_frame[5] ={0,};
    if (comm == COMM_CUSTOM_APP_DATA)
    {

        cmd_frame[0] = comm;
        cmd_frame[1] = data.hostype;
        cmd_frame[2] = 1;
        cmd_frame[3] = data.vesc_id;
        cmd_frame[4] = data.command;
        cmd_size = 5;
        float comm_value = data.set_value;
    }
    else if(comm == COMM_FORWARD_CAN)
    {
        cmd_frame[0] = comm;
        cmd_frame[1] = data.vesc_id;
        cmd_frame[2] = data.command;
        cmd_size = 3 ;
        float comm_value = data.set_value;
    }
    else
    {
        cmd_frame[0] = data.command;
        cmd_size = 1;
    }

    int value = NULL;
            switch(data.command)
            {
                case COMM_SET_DUTY :
                    value = int(data.set_value * 100000.0);
                    break;
                case COMM_SET_CURRENT:
                    value = int(data.set_value * 1000.0);
                    break;
                case COMM_SET_CURRENT_BRAKE:
                    value = int(data.set_value * 1000.0);
                    break;

                default:
                ;
            } 

    int data_list[4] ={0,};
    if (value != NULL )
    {
        data_list[0] = (value>>24)&0XFF;
        data_list[1] = (value>>16)&0XFF;
        data_list[2] = (value>>8)&0XFF;
        data_list[3] = value & 0XFF;
    }

    
    int *dataframe=merge_array(cmd_frame,cmd_size,data_list,sizeof(data_list)/sizeof(int));
    int data_len = cmd_size + 4;
    int crc = crc16(dataframe,data_len);


    packet_data[0] = start_frame;
    packet_data[1] = data_len;
    int i = 0;

    while(i<data_len)
    {
        packet_data[i+2] = dataframe[i];
        i=i+1;
    }
    packet_data[i+2] = (crc >> 8) & 0xFF; // crch
    packet_data[i+3] = crc & 0xFF;  //crcl 
    packet_data[i+4] = 3; //end_frame
        

        return packet_data;

}


int VESC_PACKET::ethercat_send_cmd(int slave ,int cmd , float value ){

    
    if(slave > ec_slavecount)
    {   
        return 0;
    }
    
    Ecat_raw temp ; 
    temp.hostype = ETHERCAT;
    temp.vesc_id = 0xFF;

    int command = 0;
    int comm_set_cmd = COMM_CUSTOM_APP_DATA;
    
    
    int hostype = ETHERCAT;
    float set_value = 0;
    
    if (cmd == 1)
    {
        temp.command = COMM_SET_DUTY;
        temp.set_value = value ;
    }
    else if (cmd == 2)
    {
        temp.command = COMM_SET_CURRENT;
        temp.set_value = value ;
    }
    else if (cmd == 3)
    {
        temp.command = COMM_SET_CURRENT_BRAKE;
        temp.set_value = value ;
    }
    else if (cmd == 4)
    {
        temp.command = COMM_SET_CURRENT;
        temp.set_value = 0 ;
    }
    else
    {
        temp.command = 0;
        temp.set_value = 0 ;
    }
    int* send_data=packet_encoding(comm_set_cmd,temp);

    for(int i = (ec_slavecount-1)*BYTE_NUM ; i<BYTE_NUM*(ec_slavecount) ;i++)
    {
         ec_slave[slave].outputs[i]=send_data[i];
    }
    
   return 1;

}

int VESC_PACKET::crc_check(int dataframe[], int len, int com_crch , int com_crcl)
{
    int crc = crc16(dataframe,len);
    int crch = ( crc >>8 ) & 0XFF;
    int crcl = crc & 0xFF;

    if (com_crch == crch && com_crcl == crcl)
    {
        return 1;
    }
    else
    {
        cout << "crc check - Fail"<< endl;
        cout << "received crch : " << com_crch << ", crcl : " << com_crcl << "calculated crch : " << crch << ", crcl : "<< crcl <<endl ;
        return 0;
    }

}

int VESC_PACKET::get_bytes(int data[], int length , float div)
{
    int raw_value = 0. ;
    double value = 0.;
    int result = 0;
    for(int i = length ; i>-1 ; i--)
        raw_value = raw_value | data[-(i+1)] << 8*i ;

    switch (length)
    {
    case 4 :
        value = status_negative_value_process_4_byte(raw_value);
        break;

    case 3 :
        value = status_negative_value_process_3_byte(raw_value);
        break;

    case 2 :
        value = status_negative_value_process_2_byte(raw_value);
        break;
    
    default:
        value = raw_value;
        break;
    }

    if (div == 1)
    {
        result = int(value);
    }
    else
    {
        result = int(value)/div;
    }

    return result;
}