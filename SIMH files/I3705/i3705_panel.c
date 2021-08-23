/* 3705_panel.c: IBM 3705 Interfaced Operator Panel

   Copyright (c) 2020, Henk Stegeman and Edwin Freekenhorst

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Charles E. Owen shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Charles E. Owen.

   ------------------------------------------------------------------------------

   This coding works in combination with an interfaced IBM 3705
   operator panel.

   RPi CPU --- I2C bus -+- MCP23017 -+- Port A <--> I or O to panel.
                        |   (0x20)   |
                        |            '- Port B <--> I or O to panel
                        |
                        '- MCP23017 -+- Port A <--> I or O to panel.
                        |   (0x21)   |
                        |            '- Port B <--> I or O to panel
                        V Max 8 devices

   The state of all switches are scanned,
   all panel lights updated.
   The interval timer triggers every 250msec a level 3 interrupt.

 ***************************************************************
*/

#undef I2C_HW                               /* No I2C hw connected */
//#define I2C_HW                                /* I2C connected */

#include <stdio.h>
#include <sched.h>
#include <signal.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "i3705_defs.h"
#include "i3705_Eregs.h"                      /* Exernal regs defs */
#include "i3705_i2c_defs.h"

extern int32 PC;
extern int32 saved_PC;
extern int32 opcode;
extern int32 Eregs_Out[];
extern int32 Eregs_Inp[];
extern int8  timer_req_L3;
extern int8  inter_req_L3;
// CCU status flags
extern int8  test_mode;
extern int8  load_state;
extern int8  wait_state;
extern int8  pgm_stop;

void sig_handler (int signo);
void timer_msec (long int msec);
int wiringPiI2CSetup(int dID0);
int wiringPiI2CReadReg8(int fd, int reg);
int wiringPiI2CWriteReg8(int fd, int reg, int data);
void MCP_init(void);

int MCP_open = OFF;          // Init flag
int fdPCB0, fdPCB1, fdPCB2, fdPCB3, fdPCB4;
int pcbID0 = 0x20;           // Outputs for Display leds
int pcbID1 = 0x21;           // Input for hex switches
int pcbID2 = 0x22;           // Outputs for display leds
int pcbID3 = 0x23;           // Input buttons, Output status leds
int pcbID4 = 0x24;           // Input 4 & 10 position rot. sw.
int rc, inp, i;
int32 hex_sw, rot_sw;

// **********************************************************
// The panel adaptor handling thread starts here...
// **********************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */

void *PNL_thread(void *arg) {
   fprintf(stderr, "\nPNL: thread started succesfully...  \n");


   // We can set one or more bits here, each one representing a single CPU
   cpu_set_t cpuset;

   // Select the CPU core we want to use
   int cpu = 2;

   CPU_ZERO(&cpuset);       // Clears the cpuset
   CPU_SET( cpu , &cpuset); // Set CPU  on cpuset

   /*
    * cpu affinity for the calling thread
    * first parameter is the pid, 0 = calling thread
    * second parameter is the size of your cpuset
    * third param is the cpuset in which your thread
    * will be placed. Each bit represents a CPU.
    */
   sched_setaffinity(0, sizeof(cpuset), &cpuset);


#ifdef I2C_HW
   /* Open the MCP23017's if not already done... */
   if (MCP_open == OFF) {
      MCP_init();
      MCP_open = ON;                           /* Set flag */
   }
#endif

   signal (SIGALRM, sig_handler);              /* Interval timer */
   timer_msec(250);             // <=== sets the 3705 interval timer

   int start_bton = RELEASED, stop_bton = RELEASED;
   int inter_bton = RELEASED, load_bton = RELEASED;
   int setad_bton = RELEASED, reset_bton = RELEASED;
   int disp_regA, disp_regB;
   int inp_h, inp_l;
   int five_lt = 0x00;

   // *******************************************************
   // Start port scanning for input and update outputs
   // *******************************************************
   while (1) {

#ifdef I2C_HW
      /* Get all input port values for later use */

      // *****************************************************
      // Hex DIAL switches
      // *****************************************************
      inp_h = wiringPiI2CReadReg8(fdPCB1, GPIOA);
      inp_l = wiringPiI2CReadReg8(fdPCB1, GPIOB);
      hex_sw = (inp_h << 8) | inp_l;
      Eregs_Inp[0x71] = hex_sw;

      // *****************************************************
      // Mode and Display/Function selector
      // *****************************************************
      inp_h = wiringPiI2CReadReg8(fdPCB4, GPIOA);
      inp_l = wiringPiI2CReadReg8(fdPCB4, GPIOB);
      rot_sw = (inp_h << 8) | inp_l;
      Eregs_Inp[0x72] = rot_sw & 0x007E;  // Read panel mode/status sw
      if (rot_sw & 0x8000)         // HJS Map Register Address ?
         Eregs_Inp[0x72] |= 0x0800;
      if (rot_sw & 0x4000)         // HJS Map Storage Address ?
         Eregs_Inp[0x72] |= 0x1000;
      // *****************************************************
      // Process buttons pressed or released...
      // *****************************************************
      inp_l = wiringPiI2CReadReg8(fdPCB3, GPIOA);
      if (inp_l & BT_START) {       /* START pressed ? */
         if (start_bton == RELEASED)
            start_bton = PRESSED;
      } else {                      /* START released ? */
         if (start_bton == ACTED)
            start_bton = RELEASED;
      }
      if (inp_l & BT_STOP) {        /* STOP pressed ? */
         if (stop_bton == RELEASED)
            stop_bton = PRESSED;
      } else {                      /* STOP released ? */
         if (stop_bton == ACTED)
            stop_bton = RELEASED;
      }
      if (inp_l & BT_INTERRUPT) {   /* INTERRUPT pressed ? */
         if (inter_bton == RELEASED)
            inter_bton = PRESSED;
      } else {                      /* INTERRUPT released ? */
         if (inter_bton == ACTED)
            inter_bton = RELEASED;
      }
      if (inp_l & BT_LOAD) {        /* LOAD pressed ? */
         if (load_bton == RELEASED)
            load_bton = PRESSED;
      } else {                      /* LOAD released ? */
         if (load_bton == ACTED)
            load_bton = RELEASED;
      }

      /* Debounce SET ADDRESS DISPLAY button when pressed */
      if (~inp_l & BT_SET_A_D) {     /* SET ADDR DISP pressed ? */
         usleep(25000);
         inp_l = wiringPiI2CReadReg8(fdPCB3, GPIOA);
         if (~inp_l & BT_SET_A_D)    /* Confirmed ? */
            setad_bton = PRESSED;
      }

//    // *****************************************************
//    //               LOAD pressed ?
//    // *****************************************************
//    if (load_bton == PRESSED) {
//       if ((ce_mode == CE_PROCESS) ||  /* Only PROCESS/INST STEP */
//           (ce_mode == CE_INST_STEP)) {
//          if (ce_mode == CE_INST_STEP) {  /* Single instruction ? */
//             cpu_si = ON;              /* Not too fast... */
//          } else {
//             cpu_si = OFF;             /* Full throttle ! */
//          }
//          cpu_state = RUN;
//          signal_condition(&cpugate);  /* Go */
//          release_lock(&cpulock);
//          dp_c &= ~PROC_CHK_LMP;       /* Processor check off */
//          cp_b &= ~STOP_LMP;           /* Stop light off      */
//          dp_c &= ~ADDR_COMP_LMP;      /* Addr comp light off */
//          dp_c &= ~IO_ATT_LMP;         /* I/O Attention off   */
//       }
//       load_bton = ACTED;              /* Action is taken */
//    }
//
//    // *****************************************************
//    //               STOP pressed ?
//    // *****************************************************
//    if (stop_bton == PRESSED) {
//       obtain_lock(&cpulock);
//       if (cpu_state != STOPPED) {
//          cpu_state = STOP;            /* Reset CPU */
//          signal_condition(&cpugate);  /* Go */
//       }
//       release_lock(&cpulock);
//       cp_b |= STOP_LMP;               /* Stop light on */
//       cpu_mc_a  = 0x00;               /* All MC lights off */
//       cpu_mc_b &= 0xF8;
//       stop_bton = ACTED;              /* Action is taken */
//    }
//
//    // *****************************************************
//    //               START pressed ?
//    // *****************************************************
//    if (start_bton == PRESSED) {
//       switch (ce_mode) {
//          case CE_INST_STEP:           /* Go... ? */
//          case CE_PROCESS:
//             obtain_lock(&cpulock);
//             if (cpu_state == STOPPED) {
//                cpu_cmd = NORMAL;
//                if (ce_mode == CE_INST_STEP) {  /* Single instruction ? */
//                   cpu_si = ON;        /* Not too fast... */
//                } else {
//                   cpu_si = OFF;       /* Full throttle ! */
//                }
//                cpu_state = RUN;
//                signal_condition(&cpugate);  /* Get going... */
//             }
//             if ((cpu_state == RUNNING) && (cpu_hpl == ON))
//                /* Continue after HPL */
//                cpu_state = RUN;
//             release_lock(&cpulock);
//             cp_b &= ~STOP_LMP;        /* Stop light off */
//             dp_c &= ~ADDR_COMP_LMP;   /* Addr comp light off */
//             start_bton = ACTED;       /* Action is taken */
//             break;
//          case CE_ALT_SAR:             /* Alter SAR ? */
//             SAR = SR;
//             IAR[level] = SAR;
//             start_bton = ACTED;       /* Action is taken */
//             break;
//          case CE_DSP_STOR:            /* Display storage ? */
//             obtain_lock(&cpulock);
//             cpu_cmd = DISP_STOR;
//             if (ce_sw & ADR_INCR_OFF) {
//                cpu_sar_incr = OFF;
//             } else {
//                cpu_sar_incr = ON;
//             }
//             if (ce_sw & STOR_TST_RUN) {
//                cpu_stor_test = ON;
//             } else {
//                cpu_stor_test = OFF;
//             }
//             cpu_state = RUN;
//             signal_condition(&cpugate);  /* Go */
//             release_lock(&cpulock);
//             cp_b &= ~STOP_LMP;        /* Stop light off */
//             start_bton = ACTED;       /* Action is taken */
//             break;
//          case CE_ALT_STOR:            /* Alter storage ? */
//             obtain_lock(&cpulock);
//             cpu_cmd = ALTER_STOR;
//             if (ce_sw & ADR_INCR_OFF) {
//                cpu_sar_incr = OFF;
//             } else {
//                cpu_sar_incr = ON;
//             }
//             if (ce_sw & STOR_TST_RUN) {
//                cpu_stor_test = ON;
//             } else {
//                cpu_stor_test = OFF;
//             }
//             cpu_state = RUN;
//             signal_condition(&cpugate);  /* Go */
//             release_lock(&cpulock);
//             cp_b &= ~STOP_LMP;        /* Stop light off */
//             start_bton = ACTED;       /* Action is taken */
//             break;
//          default:
//             break;
//       }
//    }
//
//    // *****************************************************
//    //               SET ADDRESS DISPLAY pressed ?
//    // *****************************************************
//    if (setad_bton == pressed) {
//       obtain_lock(&cpulock);
//       cpu_cmd = TERMINATE;            /* Return to SIMH */
//       cpu_state = RUN;
//       signal_condition(&cpugate);     /* Go */
//       release_lock(&cpulock);
//       setad_bton = ACTED;             /* Action is taken */
//    }
//
   // *****************************************************
   //               INTERRUPT pressed ?
   // *****************************************************
   if (inter_bton == PRESSED) {
      inter_req_L3 = ON;
      inter_bton = ACTED;                /* Action is taken */
   }
//
//    // *****************************************************
//    //               RESET pressed ?
//    // *****************************************************
//    if (reset_bton == PRESSED) {
//       /* Reset the CPU */
//       obtain_lock(&cpulock);
//       cpu_cmd = CPU_RESET;            /* Reset CPU */
//       cpu_state = RUN;
//       signal_condition(&cpugate);     /* Go */
//       release_lock(&cpulock);
//       reset_bton = ACTED;             /* Action is taken */
//    }
//
//    /* Address Compare Stop ? */
//    if (ce_sw & ADR_CMP_STOP) {        /* Address compare stop ? */
//       if (roller == SAR_HI_LO) {      /* Roller in correct pos. ? */
//          cpu_sar_cmp = ON;            /* Turn on SAR addr compare */
//       }
//    } else {
//       cpu_sar_cmp = OFF;              /* Turn it off */
//    }

      // *****************************************************
      //               Update all the panel (status) lights
      // *****************************************************
      // *** HARD_STOP

      // *** TEST
      if (test_mode == ON)
         five_lt |=  LT_TEST;
      else
         five_lt &= ~LT_TEST;

      // *** WAIT
      if (wait_state == ON)
         five_lt |=  LT_WAIT;
      else
         five_lt &= ~LT_WAIT;

      // *** PROGRAM_STOP
      if (pgm_stop == ON)
         five_lt |=  LT_PGM_STOP;
      else
         five_lt &= ~LT_PGM_STOP;

      // *** LOAD
      if (load_state == ON)
         five_lt |=  LT_LOAD;
      else
         five_lt &= ~LT_LOAD;

      // *****************************************************
      //               LAMP TEST pressed ?
      // *****************************************************
      if ((inp_l & 0x00) != 0x00) {     /* Paris by night   */
         wiringPiI2CWriteReg8(fdPCB0, GPIOA, 0xFF);
         wiringPiI2CWriteReg8(fdPCB0, GPIOB, 0xFF);
         wiringPiI2CWriteReg8(fdPCB2, GPIOA, 0xFF);
         wiringPiI2CWriteReg8(fdPCB2, GPIOB, 0xFF);
         wiringPiI2CWriteReg8(fdPCB3, GPIOB, 0x1F);

      } else {

      // *****************************************************
      //               Update Display A & B lights
      // *****************************************************
         switch (rot_sw & 0xF07E) {      /* DFS INPUTS ONLY */
            case DF_TAR_OP_REG:          /* TAR & OP Reg. */
               disp_regA = saved_PC;     /* TAR */
               disp_regB = opcode;       /* Opcode */
               break;

            case DF_STATUS:              /* Status */
               disp_regA = PC;
               disp_regB = hex_sw;       /* HJS Special */
               break;

            case DF_STOR_ADDR:           /* Storage address */
            case DF_REG_ADDR:            /* Register address */
            case DF_FUNC_1:              /* Function 1-6 */
            case DF_FUNC_2:              /* Function 1-6 */
            case DF_FUNC_3:              /* Function 1-6 */
            case DF_FUNC_4:              /* Function 1-6 */
            case DF_FUNC_5:              /* Function 1-6 */
            case DF_FUNC_6:              /* Function 1-6 */
               disp_regA = Eregs_Out[0x71];
               disp_regB = Eregs_Out[0x72];
               break;
            default:
               disp_regA = 0x0000;
               disp_regB = 0x0000;
               break;
         }

         wiringPiI2CWriteReg8(fdPCB0, GPIOA, (disp_regA & 0xFF));
         wiringPiI2CWriteReg8(fdPCB2, GPIOA, (disp_regA >> 8) & 0xFF);
         wiringPiI2CWriteReg8(fdPCB0, GPIOB, (disp_regB & 0xFF));
         wiringPiI2CWriteReg8(fdPCB2, GPIOB, (disp_regB >> 8) & 0xFF);

         wiringPiI2CWriteReg8(fdPCB3, GPIOB, five_lt);
         // End of panel check and updates
      }
#endif
// usleep(1000);
   }             /* End while (1) loop */
}

// *****************************************************
// Interval timer definition
// *****************************************************
void timer_msec (long int msec) {
   struct itimerval timer1;

   timer1.it_interval.tv_usec = (1000 * msec);
   timer1.it_interval.tv_sec = 0;
   timer1.it_value.tv_usec = (1000 * msec);
   timer1.it_value.tv_sec = 0;

   setitimer (ITIMER_REAL, &timer1, NULL);
}

// Kick the 3705 100msec timer...
void sig_handler (int signo) {
   if (test_mode == OFF)
      timer_req_L3 = ON;
}

// *****************************************************
// Setup all MCP23017 i2c devices  */
// *****************************************************
void MCP_init() {

   //*** initialize fdPCB0
   fdPCB0 = wiringPiI2CSetup(pcbID0);
   rc = wiringPiI2CWriteReg8(fdPCB0, IOCON, 0x00);
   if (rc < 0)
      printf("Error connecting with i2c device: 0x%02X ! \n\r",
             pcbID0);
   else
      printf("i2c device: 0x%02X successfully opened. \n\r",
             pcbID0);

   //*** initialize fdPCB1
   fdPCB1 = wiringPiI2CSetup(pcbID1);
   rc = wiringPiI2CWriteReg8(fdPCB1, IOCON, 0x00);
   if (rc < 0)
      printf("Error connecting with i2c device: 0x%02X ! \n\r",
             pcbID1);
   else
      printf("i2c device: 0x%02X successfully opened. \n\r",
             pcbID1);

   //*** initialize fdPCB2
   fdPCB2 = wiringPiI2CSetup(pcbID2);
   rc = wiringPiI2CWriteReg8(fdPCB2, IOCON, 0x00);
   if (rc < 0)
      printf("Error connecting with i2c device: 0x%02X ! \n\r",
             pcbID2);
   else
      printf("i2c device: 0x%02X successfully opened. \n\r",
             pcbID2);

   //*** initialize fdPCB3
   fdPCB3 = wiringPiI2CSetup(pcbID3);
   rc = wiringPiI2CWriteReg8(fdPCB3, IOCON, 0x00);
   if (rc < 0)
      printf("Error connecting with i2c device: 0x%02X ! \n\r",
             pcbID3);
   else
      printf("i2c device: 0x%02X successfully opened. \n\r",
             pcbID3);

   //*** initialize fdPCB4
   fdPCB4 = wiringPiI2CSetup(pcbID4);
   rc = wiringPiI2CWriteReg8(fdPCB4, IOCON, 0x00);
   if (rc < 0)
      printf("Error connecting with i2c device: 0x%02X ! \n\r",
             pcbID4);
   else
      printf("i2c device: 0x%02X successfully opened. \n\r",
             pcbID4);

   /**************************************************/
   /* Configure all 8 bits ports for input or output */
   /**************************************************/
   rc = wiringPiI2CWriteReg8(fdPCB0, IODIRA, 0x00);    // Outputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID0, IODIRA);
   rc = wiringPiI2CWriteReg8(fdPCB0, IODIRB, 0x00);    // Outputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID0, IODIRB);
   rc = wiringPiI2CWriteReg8(fdPCB1, IPOLA, 0xFF);     // Inverse
   rc = wiringPiI2CWriteReg8(fdPCB1, IODIRA, 0xFF);    // Inputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID1, IODIRA);
   rc = wiringPiI2CWriteReg8(fdPCB1, IPOLB, 0xFF);     // Inverse
   rc = wiringPiI2CWriteReg8(fdPCB1, IODIRB, 0xFF);    // Inputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID1, IODIRB);

   rc = wiringPiI2CWriteReg8(fdPCB2, IODIRA, 0x00);    // Outputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID2, IODIRA);
   rc = wiringPiI2CWriteReg8(fdPCB2, IODIRB, 0x00);    // Outputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID2, IODIRB);

   rc = wiringPiI2CWriteReg8(fdPCB3, IODIRA, 0xFF);    // Inputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID3, IODIRA);
   rc = wiringPiI2CWriteReg8(fdPCB3, IODIRB, 0x00);    // Outputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID3, IODIRB);

   rc = wiringPiI2CWriteReg8(fdPCB4, IPOLA, 0xFF);     // Inverse
   rc = wiringPiI2CWriteReg8(fdPCB4, IODIRA, 0xFF);    // Inputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID4, IODIRA);
   rc = wiringPiI2CWriteReg8(fdPCB4, IPOLB, 0xFF);     // Inverse
   rc = wiringPiI2CWriteReg8(fdPCB4, IODIRB, 0xFF);    // Inputs
   if (rc < 0)
      printf("Error writing to i2c dev: 0x%02X, reg: 0x%02X ! \n\r",
             pcbID4, IODIRB);
}
