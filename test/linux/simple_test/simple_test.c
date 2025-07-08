/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500
#define SLAVEINDEX                  1
char IOmap[4096];
OSAL_THREAD_HANDLE thread1,thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

/*** FUNCTION PROTOTYPES      ***/
bool SOEM_Write8(uint16_t Index,uint8_t SubIndex,uint8_t Value);
bool SOEM_Write16(uint16_t Index,uint8_t SubIndex,uint16_t Value);
bool SOEM_Write32(uint16_t Index,uint8_t SubIndex,uint32_t Value);
uint8_t SOEM_Read8(uint16_t Index,uint8_t SubIndex);
uint16_t SOEM_Read16(uint16_t Index,uint8_t SubIndex);
uint32_t SOEM_Read32(uint16_t Index,uint8_t SubIndex);
void PrintError(const char *fmt,...);
#if 0
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
/*******************************************************************************
 * NAME:
 *    PrintError
 *
 * SYNOPSIS:
 *    void PrintError(const char *fmt,...);
 *
 * PARAMETERS:
 *    fmt [I] -- The printf() formating string
 *    ... [I] -- printf() args
 *
 * FUNCTION:
 *    This function prints an error in red with the word ERROR: before it.
 *    It also adds a \n to the end of the line.
 *
 * RETURNS:
 *    NONE
 *
 * NOTES:
 *    Outputs the stdout.
 *
 * SEE ALSO:
 *
 ******************************************************************************/
void PrintError(const char *fmt,...)
{
    va_list args;

    va_start(args,fmt);
    printf("\33[1;31mERROR:");  // Red error
    vprintf(fmt,args);
    printf("\33[m\n");          // Normal
    va_end(args);
}
/*******************************************************************************
 * NAME:
 *    SOEM_Write8
 *
 * SYNOPSIS:
 *    bool SOEM_Write8(uint16_t Index,uint8_t SubIndex,uint8_t Value);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to write to
 *    SubIndex [I] -- The sub index for this register
 *    Value [I] -- The value to write.
 *
 * FUNCTION:
 *    This function uses the ec_SDOwrite() to write to the EtherCat motor.
 *
 * RETURNS:
 *    true -- Things worked out
 *    false -- There was an error.  Error was printed to the screen.
 *
 * SEE ALSO:
 *    SOEM_Write16(), SOEM_Write32()
 ******************************************************************************/
bool SOEM_Write8(uint16_t Index,uint8_t SubIndex,uint8_t Value)
{
    if(ec_SDOwrite(SLAVEINDEX,Index,SubIndex,false,sizeof(Value),&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Write8(0x%04X,0x%x)",Index,Value);
        return false;
    }
    return true;
}

/*******************************************************************************
 * NAME:
 *    SOEM_Write16
 *
 * SYNOPSIS:
 *    bool SOEM_Write16(uint16_t Index,uint8_t SubIndex,uint8_t Value);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to write to
 *    SubIndex [I] -- The sub index for this register
 *    Value [I] -- The value to write.
 *
 * FUNCTION:
 *    This function uses the ec_SDOwrite() to write to the EtherCat motor.
 *
 * RETURNS:
 *    true -- Things worked out
 *    false -- There was an error.  Error was printed to the screen.
 *
 * SEE ALSO:
 *    SOEM_Write8(), SOEM_Write32()
 ******************************************************************************/
bool SOEM_Write16(uint16_t Index,uint8_t SubIndex,uint16_t Value)
{
    if(ec_SDOwrite(SLAVEINDEX,Index,SubIndex,false,sizeof(Value),&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Write16(0x%04X,0x%x)",Index,Value);
        return false;
    }
    return true;
}

/*******************************************************************************
 * NAME:
 *    SOEM_Write32
 *
 * SYNOPSIS:
 *    bool SOEM_Write32(uint16_t Index,uint8_t SubIndex,uint8_t Value);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to write to
 *    SubIndex [I] -- The sub index for this register
 *    Value [I] -- The value to write.
 *
 * FUNCTION:
 *    This function uses the ec_SDOwrite() to write to the EtherCat motor.
 *
 * RETURNS:
 *    true -- Things worked out
 *    false -- There was an error.  Error was printed to the screen.
 *
 * SEE ALSO:
 *    SOEM_Write16(), SOEM_Write8()
 ******************************************************************************/
bool SOEM_Write32(uint16_t Index,uint8_t SubIndex,uint32_t Value)
{
    if(ec_SDOwrite(SLAVEINDEX,Index,SubIndex,false,sizeof(Value),&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Write32(0x%04X,0x%x)",Index,Value);
        return false;
    }
    return true;
}

/*******************************************************************************
 * NAME:
 *    SOEM_Read8
 *
 * SYNOPSIS:
 *    uint8_t SOEM_Read8(uint16_t Index,uint8_t SubIndex);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to read from
 *    SubIndex [I] -- The sub index for this register
 *
 * FUNCTION:
 *    This function uses the ec_SDOread() to read from the motor.
 *
 * RETURNS:
 *    The value read or 0 if there was an error.
 *
 * SEE ALSO:
 *    SOEM_Read16(), SOEM_Read32()
 ******************************************************************************/
uint8_t SOEM_Read8(uint16_t Index,uint8_t SubIndex)
{
    uint8_t Value;
    int Size;

    Value=0;
    Size=sizeof(Value);
    if(ec_SDOread(SLAVEINDEX,Index,SubIndex,false,&Size,&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Read8(0x%04X,0x%x)",Index,SubIndex);
        return 0;
    }
    return Value;
}

/*******************************************************************************
 * NAME:
 *    SOEM_Read16
 *
 * SYNOPSIS:
 *    uint8_t SOEM_Read16(uint16_t Index,uint8_t SubIndex);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to read from
 *    SubIndex [I] -- The sub index for this register
 *
 * FUNCTION:
 *    This function uses the ec_SDOread() to read from the motor.
 *
 * RETURNS:
 *    The value read or 0 if there was an error.
 *
 * SEE ALSO:
 *    SOEM_Read8(), SOEM_Read32()
 ******************************************************************************/
uint16_t SOEM_Read16(uint16_t Index,uint8_t SubIndex)
{
    uint16_t Value;
    int Size;

    Value=0;
    Size=sizeof(Value);
    if(ec_SDOread(SLAVEINDEX,Index,SubIndex,false,&Size,&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Read16(0x%04X,0x%x)",Index,SubIndex);
        return 0;
    }
    return Value;
}

/*******************************************************************************
 * NAME:
 *    SOEM_Read32
 *
 * SYNOPSIS:
 *    uint8_t SOEM_Read32(uint16_t Index,uint8_t SubIndex);
 *
 * PARAMETERS:
 *    Index [I] -- The EtherCat register to read from
 *    SubIndex [I] -- The sub index for this register
 *
 * FUNCTION:
 *    This function uses the ec_SDOread() to read from the motor.
 *
 * RETURNS:
 *    The value read or 0 if there was an error.
 *
 * SEE ALSO:
 *    SOEM_Read16(), SOEM_Read8()
 ******************************************************************************/
uint32_t SOEM_Read32(uint16_t Index,uint8_t SubIndex)
{
    uint32_t Value;
    int Size;

    Value=0;
    Size=sizeof(Value);
    if(ec_SDOread(SLAVEINDEX,Index,SubIndex,false,&Size,&Value,
            EC_TIMEOUTRXM)==0)
    {
        PrintError("SOEM_Read32(0x%04X,0x%x)",Index,SubIndex);
        return 0;
    }
    return Value;
}
int setup_drive(uint16_t slave) {
	   (void)slave;
	   SOEM_Write8(0x1C12,0x00,0);
	   SOEM_Write8(0x1C13,0x00,0);
	   SOEM_Write8(0x1A00,0x00,0);
	   SOEM_Write32(0x1A00,0x01,0x60640020);
	   SOEM_Write32(0x1A00,0x02,0x606C0020);
	   SOEM_Write32(0x1A00,0x03,0x60770010);
	   SOEM_Write32(0x1A00,0x04,0x60410010);
	   SOEM_Write8(0x1A00,0x00,4);
	   SOEM_Write8(0x1A01,0x00,0);
	   SOEM_Write32(0x1A01,0x01,0x60640020);
	   SOEM_Write32(0x1A01,0x02,0x606C0020);
	   SOEM_Write32(0x1A01,0x03,0x60410010);
	   SOEM_Write8(0x1A01,0x00,3);
	   SOEM_Write8(0x1600,0x00,0);
	   SOEM_Write32(0x1600,0x01,0x607A0020);
	   SOEM_Write32(0x1600,0x02,0x60FF0020);
	   SOEM_Write32(0x1600,0x03,0x60710010);
	   SOEM_Write32(0x1600,0x04,0x60400010);
	   SOEM_Write8(0x1600,0x00,4);
	   SOEM_Write8(0x1601,0x00,0);
	   SOEM_Write32(0x1601,0x01,0x607A0020);
	   SOEM_Write32(0x1601,0x02,0x60FF0020);
	   SOEM_Write32(0x1601,0x03,0x60400010);
	   SOEM_Write8(0x1601,0x00,3);
	   SOEM_Write32(0x1C12,0x01,0x1601);
	   SOEM_Write8(0x1C12,0x00,0x1);
      SOEM_Write32(0x1C13,0x01,0x1A01);
	   SOEM_Write8(0x1C13,0x00,0x1);
      SOEM_Write8(0x6060,0x00,8);
      SOEM_Write32(0x1C32,0x02,2000000);
	   return 0;

}

OSAL_THREAD_FUNC motion_ctrl( void *ptr ) {
   (void)ptr;                  /* Not used */
   while(1) {
      sleep(2);
      servo_out->tgt_pos = servo_in->act_pos + 100000;
   }
}

void simpletest(char *ifname)
{
    int i,  oloop, iloop, chk,cnt;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         ec_slave[1].PO2SOconfig = setup_drive;

         if (forceByteAlignment)
         {
            ec_config_map_aligned(&IOmap);
         }
         else
         {
            ec_config_map(&IOmap);
         }

         ec_configdc();
         ec_readstate();
         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         printf("%d slaves found and configured.\n",ec_slavecount);
         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
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

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            SOEM_Write8(0x6060,0x00,8);

            servo_out->ctrl_word = 6;
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            servo_out->ctrl_word = 7;
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            servo_out->ctrl_word = 0x0F;
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            //uint32_t temp_cnt = 10000;
            osal_thread_create(&thread2, 123, &motion_ctrl, NULL);
            servo_out->tgt_pos = servo_in->act_pos + 100000;

            /*while(temp_cnt--) {

               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

               usleep(500);

            }

            printf("State %d status_word %x\n\r",ec_slave[1].state, servo_in->sts_word);
            printf("act_vel %d act_pos %d sts_word %x\n",servo_in->act_vel, servo_in->act_pos, servo_in->sts_word); */
            printf("Operational state reached for all slaves.\n");

            inOP = TRUE;
            /* cyclic loop */
            for(;;)
            {

               //SOEM_Write8(0x6060,0x00,3);
               //servo_out->tgt_pos = 100000;
               //servo_out->tgt_vel = 0x100000;
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);
               //if (servo_in->act_pos <= servo_out->tgt_pos)
               //   printf("act_vel %d act_pos %d sts_word %x\n",servo_in->act_vel, servo_in->act_pos, servo_in->sts_word); 
               /*     if(wkc >= expectedWKC)
                    {
                        printf("Processdata cycle %4d, WKC %d , O:", i=0, wkc);

                        for(j = 0 ; j < oloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].outputs + j));
                        }

                        printf(" I:");
                        for(j = 0 ; j < iloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].inputs + j));
                        }
                        printf(" T:%"PRId64"\r",ec_DCtime);
                        needlf = TRUE;
                    }*/
                  osal_usleep(1000);
		            //IOmap[0] = ~IOmap[0];

                }
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
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

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

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      ec_adaptert * adapter = NULL;
      ec_adaptert * head = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
   }

   printf("End program\n");
   return (0);
}
