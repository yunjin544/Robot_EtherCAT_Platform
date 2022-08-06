#include <iostream>
#include "VESCular_Command.h"
#include "array_sum.h"
#include "VESCular_crc.h"

#define BYTE_NUM 32

using namespace std;

struct Ecat_raw
{
    int hostype;
    int command;
    int vesc_id;
    float set_value;
};


int* ethercat_send_cmd( char *cmd , float value );
int* packet_encoding( int comm , Ecat_raw data );



int* packet_encoding( int comm , Ecat_raw data ){
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

int* ethercat_send_cmd( char *cmd , float value ){
    Ecat_raw temp ; 
    temp.hostype = ETHERCAT;
    temp.vesc_id = 0xFF;

    int command = 0;
    int comm_set_cmd = COMM_CUSTOM_APP_DATA;
    
    
    int hostype = ETHERCAT;
    float set_value = 0;
    
    if (cmd == "duty")
    {
        temp.command = COMM_SET_DUTY;
        temp.set_value = value ;
    }
    else if (cmd == "current")
    {
        temp.command = COMM_SET_CURRENT;
        temp.set_value = value ;
    }
    else if (cmd == "current_brake")
    {
        temp.command = COMM_SET_CURRENT_BRAKE;
        temp.set_value = value ;
    }
    else if (cmd == "release")
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
   return send_data;

}