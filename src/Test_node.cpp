#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include "ethercat.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#define CLOCK_RES 1e-9 
#define EC_TIMEOUTMON 500
#define LOOP_PERIOD 1e6
#define NSEC_PER_SEC 1000000000

char IOmap[256];
int expectedWKC;
int chk;
volatile int wkc;
OSAL_THREAD_HANDLE thread1;
boolean LED_Control = TRUE;
boolean needlf;
boolean inOP;
uint8 currentgroup = 0;
int32 toff, gl_delta;
RT_TASK loop_task;
struct timespec time_stamp[2000];
int tick[2000];
char flag =0;

int cycle ;
int deltat, tmax = 0;
int DCdiff;
struct timeval tv, t1, t2;



void ECat_init(char *ifname, char *ifname2);
void ECat_PDO_LOOP(void *arg);
void Ecatcheck(void *ptr);

int main(int argc, char **argv)
{
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0,&set);
   cycle = atoi(argv[3]);
ros::init(argc,argv,"robot_Ecat_master");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("ros_communication", 1000);
ros::Rate loop_rate(1000);

ECat_init(argv[1],argv[2]);
osal_thread_create(&thread1, 128000, (void*)&Ecatcheck, NULL);
rt_task_create(&loop_task, "Ecat Loop", 0, 99, 0);
rt_task_set_affinity(&loop_task,&set);
rt_task_start(&loop_task, &ECat_PDO_LOOP, 0);
int count = 0;
int save_i = 0;
   std_msgs::Int32 msg;

 while (ros::ok())
 {
	msg.data = (int)(ec_slave[0].inputs[0]);


	chatter_pub.publish(msg);
	    ros::spinOnce();
	   loop_rate.sleep();
	++count;
	}
  return 0;
}


void ECat_init(char *ifname, char *ifname2)
{

   if (ec_init_redundant(ifname, ifname2))
   {
      printf("DRCL EtherCAT Master Init Succeeded!! ( on %s , %s)\n",ifname,ifname2);

      if (ec_config_init(FALSE) > 0)
      {
         ec_config_map(&IOmap);
         printf("%d slaves found and configured.\n", ec_slavecount);

         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

         ec_configdc();

         ec_readstate();

         printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         
         printf("Request operational state for all slaves\n");
         
         ec_slave[0].state = EC_STATE_OPERATIONAL;

         ec_writestate(0);
         ec_statecheck(0, EC_STATE_OPERATIONAL, 5*EC_TIMEOUTSTATE);

         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
         }
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);

         // ec_send_processdata();
         // ec_receive_processdata(EC_TIMEOUTRET);
         
         // chk = 200;
         // ec_send_processdata();
         // ec_receive_processdata(EC_TIMEOUTRET);
         
         // while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL))
         //    ;
      }
      else
      {
         printf("No socket connection on %s\nExcecute as root\n",ifname);
      }
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}

void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

void ECat_PDO_LOOP(void *arg)
{
   RT_TASK *curtask;
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;
   struct timespec   begin, end;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = cycle * 1000; /* cycletime in ns */
   
   
   //rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
   ec_send_processdata();
   RTIME time;
   unsigned int i = 0;
   while (1)
   {  //clock_gettime(CLOCK_MONOTONIC, &begin);
      add_timespec(&ts, cycletime + toff);
      time = (ts.tv_sec) + (ts.tv_nsec);
      rt_task_sleep_until(time);
     /* Start Loop - Write your Realtime Program*/
      wkc = ec_receive_processdata(EC_TIMEOUTRET);


      ec_slave[0].outputs[0] = !(ec_slave[0].outputs[0]);

            
      ec_send_processdata();
      //rt_task_wait_period(NULL);
      clock_gettime(CLOCK_MONOTONIC, &end);

      //long ptime = (end.tv_sec - begin.tv_sec) + (end.tv_nsec - begin.tv_nsec);
      //double time_s = (double)ptime/1000000000;
      //rt_printf("Frequency (): %4.2lf\r",1/time_s);
      /* End Loop */
   }
}

void Ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
   }
}
