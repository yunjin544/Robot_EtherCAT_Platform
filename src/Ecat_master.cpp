#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#define CLOCK_RES 1e-9 
#define EC_TIMEOUTMON 500
#define LOOP_PERIOD 1e6
#define NSEC_PER_SEC 1000000000
#define stack64k (64 * 1024)

char IOmap[256];
OSAL_THREAD_HANDLE thread1, thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int chk;
int64 toff, gl_delta;

//void ECat_init(char *ifname, char *ifname2);
void ECat_init(char *ifname);
void ECat_PDO_LOOP(void *arg);
void Ecatcheck(void *ptr);

int main(int argc, char **argv)
{
   int ctime = LOOP_PERIOD ;
ros::init(argc,argv,"robot_Ecat_master");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("ros_communication", 1000);
ros::Rate loop_rate(2000);

//ECat_init("rteth0","rteth1");
ECat_init(argv[1]);

osal_thread_create(&thread1, stack64k*4, (void*)&Ecatcheck, NULL);
osal_thread_create_rt(&thread2, stack64k * 2,(void*)&ECat_PDO_LOOP, &ctime);
int count = 0;
int save_i = 0;
 while (ros::ok())
 {

	std_msgs::String msg;
	std::stringstream ss;
	ss << (int)(ec_slave[0].inputs[0]) << "<-variable resistance level  "; 
	msg.data = ss.str();

	//printf("%s\r", msg.data.c_str());
	
	chatter_pub.publish(msg);
	    ros::spinOnce();
	   loop_rate.sleep();
	++count;
	}
  return 0;
}


OSAL_THREAD_FUNC ECat_init(char *ifname)
{

   if (ec_init(ifname))
   {
      printf("Starting Ecat DRCL Master Test\n");

      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);
      }

      ec_config_map(&IOmap);
      printf("Slaves mapped, state to SAFE_OP.\n");
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

      printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_writestate(0);
      chk = 200;
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL))
         ;
   }
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

OSAL_THREAD_FUNC_RT ECat_PDO_LOOP(void *arg)
{
   struct timespec   ts, tleft;
   struct timespec   begin, end;

   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = *(int*)arg * 1000; /* cycletime in ns */
   toff = 0;
   ec_send_processdata();

   while (1)
   {  
      clock_gettime(CLOCK_MONOTONIC, &begin);
      add_timespec(&ts, cycletime);
   
      //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
     /* Start Loop - Write your Realtime Program*/
      wkc = ec_receive_processdata(EC_TIMEOUTRET);

      ec_slave[0].outputs[0] = !(ec_slave[0].outputs[0]);
   
      ec_send_processdata();

      
      osal_usleep(500);
      clock_gettime(CLOCK_MONOTONIC, &end);

      long time = (end.tv_sec - begin.tv_sec) + (end.tv_nsec - begin.tv_nsec);
      double time_s = (double)time/1000000000;
      rt_printf("Time (Second): %4.2lf\r",1/time_s);
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
