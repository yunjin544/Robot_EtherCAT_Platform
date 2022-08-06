#include "DRCL_Ecat.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "VESCular_ethercat.h"

#define CLOCK_RES 1e-9 
#define EC_TIMEOUTMON 500
#define LOOP_PERIOD 5e5
#define NSEC_PER_SEC 1000000000

char IOmap[512];
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
float duty_ratio =0.0 ;

void ECat_init(char *ifname, char *ifname2);
void ECat_PDO_LOOP(void *arg);
void Ecatcheck(void *ptr);


void chatterCallback(const std_msgs::Int32& msg)
{
   if (msg.data< 100 && msg.data > -100)
   {
      duty_ratio = float(msg.data)/100.0 ;
  
   }
  
}

int main(int argc, char **argv)
{


	// cpu_set_t set;
	// CPU_ZERO(&set);
	// CPU_SET(2,&set);

ros::init(argc,argv,"robot_Ecat_master");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("ros_communication", 1000);
ros::Subscriber sub = n.subscribe("duty", 10, chatterCallback);
ros::Rate loop_rate(1000);

ECat_init(argv[1],argv[2]);
osal_thread_create(&thread1, 128000, (void*)&Ecatcheck, NULL);
rt_task_create(&loop_task, "Ecat Loop", 0, 99, 0);
// rt_task_set_affinity(&loop_task,&set);
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




void ECat_PDO_LOOP(void *arg)
{
   RT_TASK *curtask;
   rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
   unsigned int i = 0;
   while (1)
   {
     /* Start Loop - Write your Realtime Program*/
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      int* send_data =ethercat_send_cmd("duty",duty_ratio);

      for(int i = 0 ; i<BYTE_NUM ;i++)
      {
         ec_slave[1].outputs[i]=send_data[i];
      }
      

      rt_printf("%d\r",i);  
      ec_send_processdata();
      rt_task_wait_period(NULL);
      
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
