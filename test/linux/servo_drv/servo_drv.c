/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : servo_init_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

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
#include <inttypes.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

#if 1
typedef struct PACKED {
	uint16_t ctrl_word;
	uint32_t tgt_pos;
}tx_pdo_t;

typedef struct PACKED {
	uint16_t sts_word;
	uint32_t act_pos;
}rx_pdo_t;
#else
typedef struct PACKED {
	uint32_t tgt_pos;
   uint32_t tgt_vel;
   uint16_t ctrl_word;
}tx_pdo_t;

typedef struct PACKED {
	uint32_t act_pos;
   uint32_t act_vel;
   uint16_t sts_word;
}rx_pdo_t;
#endif
tx_pdo_t *servo_out;
rx_pdo_t *servo_in;

void servo_init(char *ifname)
{
   int cnt, i, oloop, iloop;

   printf("Starting Redundant test\n");

   /* initialise SOEM, bind socket to ifname */

   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */
      if ( ec_config(FALSE, &IOmap) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

         /* configure DC options for every DC capable slave found in the list */
         ec_configdc();

         /* read indevidual slave state and store in ec_slave[] */
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                  ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            printf("         Out:%p,%4d In:%p,%4d\n",
                  ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
            
               servo_out = (tx_pdo_t*)ec_slave[cnt].outputs;
               servo_in = (rx_pdo_t*)ec_slave[cnt].inputs;
            
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);
         /* activate cyclic process data */
         dorun = 1;
         /* wait for all slaves to reach OP state */
         ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* acyclic loop 5000 x 20ms = 10s */
            for(i = 1; i <= 5000; i++)
            {
               /*printf("Processdata cycle %5d , Wck %3d, DCtime %12"PRId64", dt %12"PRId64", O:",
                  dorun, wkc , ec_DCtime, gl_delta);*/
               /*for(j = 0 ; j < oloop; j++)
               {
                  printf(" %x", *(ec_slave[0].outputs + j));
               }
               printf(" I:");
               for(j = 0 ; j < iloop; j++)
               {
                  printf(" %x\n", *(ec_slave[0].inputs + j));
               }*/
	 	printf("%d\n\r", servo_in->act_pos);
               fflush(stdout);
               osal_usleep(2000);
            }
            dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
             ec_readstate();
             for(i = 1; i<=ec_slavecount ; i++)
             {
                 if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                 {
                     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                 }
             }
         }
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End redundant test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
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

OSAL_THREAD_FUNC inputcheck( void *ptr )
{
	(void) ptr;
	int data;
	while(1) {
		scanf("%d", &data);
		printf("data =============== %d\n", data);
		servo_out->tgt_pos = data;
		ec_send_processdata();
          	ec_receive_processdata(EC_TIMEOUTRET);
	}
}
/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;

   printf("OP = %d\n", inOP);
   while (!inOP);
   printf("Slave in OP\n");
   
   servo_out->ctrl_word = 6;
   ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          servo_out->ctrl_word = 7;
   ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          servo_out->ctrl_word = 0x0F;
   ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          servo_out->tgt_pos = 10000;
   ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
            printf("State %d status_word %d\n\r",ec_slave[0].state, servo_in->sts_word);

   while(1)
   {
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
         wkc = ec_receive_processdata(EC_TIMEOUTRET);

         dorun++;

         /* if we have some digital output, cycle */
         //if( servo_out->tgt_pos ) servo_out->tgt_pos = (uint8) ((dorun / 16) & 0xff);

         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         ec_send_processdata();
      }
   }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;

    (void) ptr;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
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
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
   int ctime;

   printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

   if (argc > 2)
   {
      dorun = 0;
      ctime = atoi(argv[2]);

      /* create RT thread */
      osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*) &ctime);

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread2, stack64k * 4, &inputcheck, NULL);

      /* start acyclic part */
      servo_init(argv[1]);
   }
   else
   {
      printf("Usage: servo_drv ifname1 cycletime\nifname = eth0 for example\ncycletime in us\n");
   }

   printf("End program\n");

   return (0);
}
