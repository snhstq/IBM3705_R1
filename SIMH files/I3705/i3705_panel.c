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
#include <string.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <sys/time.h>
#include <sys/syscall.h>
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

#define IAC             255     /* Interpret as Command              */
#define EOR_MARK        239     /* End of record marker */
#define MAXPORTS 4

int rpfd;
int lpfd;
int        sockopt;         /* Used for setsocketoption          */
int rc,len;                 /* return code from various rtns     */
//struct sockaddr_in  sin, *sin2;     /* bind socket address structure     */
//struct ifaddrs *nwaddr, *ifa;      /* interface address structure       */
char *ipaddr;
char buf[8192];
char buft[256];
char defpos[6] = {"\x28\x41\xF0\x28\x42\xF4"};
char revred[6] = {"\x28\x41\xF2\x28\x42\xF2"};
char revgrn[6] = {"\x28\x41\xF2\x28\x42\xF4"};

char dbb0b0[3] = {"\x11\x4C\xC9"};
char dbb0b1[3] = {"\x11\x4D\xD9"};
char dbb0b2[3] = {"\x11\x4E\xE9"};
char dbb0b3[3] = {"\x11\x4F\xF9"};
char active[17]   = {"\x28\x41\xF2\x28\x42\xF4\xC1\xC3\xE3\xC9\xE5\xC5\x28\x41\xF0\x40\x40"};
char enabled[14]  = {"\x28\x41\xF0\x28\x42\xF1\xC5\xD5\xC1\xC2\xD3\xC5\xC4\x40"};
char disabled[14] = {"\x28\x41\xF0\x28\x42\xF2\xC4\xC9\xE2\xC1\xC2\xD3\xC5\xC4"};
char ca1a[3] = {"\x11\xD9\xC6"};
char ca1b[3] = {"\x11\xD9\xD3"};
char ca2a[3] = {"\x11\xD9\xE2"};
char ca2b[3] = {"\x11\xD9\x6F"};
extern struct IO3705 {
   char CA_id;
   int CA_active;
   int CA_socket[2];
   uint16_t CA_mask;
   int addrlen[MAXPORTS];
   int bus_socket[2];
   int tag_socket[2];
   int abswitch;
   uint16_t devnum;
   uint8_t buffer[1024];   // Data buffer of 1K
   uint32_t bufferl;       // Received data length
   struct sockaddr_in address[2];
   } *iob1, *iob2;

uint8_t                 class;          /* D=3270, P=3287, K=3215/1052 */
uint8_t                 model;          /* 3270 model (2,3,4,5,X)    */
uint8_t                 extended;       /* Extended attributes (Y,N) */
uint16_t                devnum;         /* Requested device number   */
char                    group[16];      /* Console group             */
int row,col,pnlmsgl;
uint16_t bfa;
unsigned char rowcol[2];
char                    pnlmsg[256];    /* panel message             */

extern int send_packet (int rpfd, uint8_t *buf, int len, char *caption);
extern int negotiate(int rpfd, uint8_t *class, uint8_t *model, uint8_t *extatr, uint16_t *devn,char *group);
extern uint8_t prt_host_to_guest( uint8_t *pnlmsgi,  uint8_t *pnlmsgo, const uint ilength  );
char* buf3270 (int row, int col);


// **********************************************************
// The panel adaptor handling thread starts here...
// **********************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */

void *PNL_thread(void *arg) {
   fprintf(stderr, "\nPNL: thread %d started succesfully...  \n",syscall(SYS_gettid));

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
   timer_msec(100);             // <=== sets the 3705 interval timer

   int start_bton = RELEASED, stop_bton = RELEASED;
   int inter_bton = RELEASED, load_bton = RELEASED;
   int setad_bton = RELEASED, reset_bton = RELEASED;
   int disp_regA, disp_regB;
   int inp_h, inp_l;
   int five_lt = 0x00;

   struct sockaddr_in  sin, *sin2;     /* bind socket address structure     */
   struct ifaddrs *nwaddr, *ifa;      /* interface address structure       */

    getifaddrs(&nwaddr);      /* get network address */
    for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name,"lo")) {
           sin2 = (struct sockaddr_in *) ifa->ifa_addr;
           ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
           if (strcmp(ifa->ifa_name,"eth")) break;
        }
    }
    printf("\nPNL: Using network Address %s on %s for 3270 panel connections\n",ipaddr,ifa->ifa_name);
    rpfd = 0;
    lpfd=socket(AF_INET,SOCK_STREAM,0);


    /* Reuse the address regardless of any */
    /* spurious connection on that port    */
    sockopt=1;
    setsockopt(lpfd,SOL_SOCKET,SO_REUSEADDR,(void*)&sockopt,sizeof(sockopt));

    /* Bind the socket */
    sin.sin_family=AF_INET;
    sin.sin_addr.s_addr = inet_addr(ipaddr);
    sin.sin_port=htons(32701);
    rc=bind(lpfd,(struct sockaddr *)&sin,sizeof(sin));
    if (rc < 0)
    {
        printf("\nPNL: Socket bind failed\n");
        return NULL;
    }
    rpfd = 0;
        /* Start the listen */
    rc = listen(lpfd, 1);
    if (rc < 0)
    {
        //WRMSG(HHC01000, "E",SSID_TO_LCSS(ca->dev->ssid),devnum,"listen()",strerror(HSO_errno));
        return NULL;
    }

   // *******************************************************
   // Start port scanning for input and update outputs
   // *******************************************************
   while (1) {
/********************************************************************/
/* Until there is a 3270 connection, nothing happens                */
/********************************************************************/

  if (rpfd < 1) {
     printf("\nPNL: Waiting for 3270 connection\n");

        rpfd=accept(lpfd,NULL,0);

    /* Negotiate telnet parameters */
    rc = negotiate (rpfd, &class, &model, &extended, &devnum, group);
    if (rc != 0)
    {
       printf("\nPNL: 3270 negotiation failed; rc = %d\n",rc);
        close (rpfd);
        rpfd = 0;
       // if (clientip) free(clientip);
        return 0;
    }
    else printf("\rPNL: 3270 negotiation OK\n");

/*********************************************************************/
/* Build the initial 3705 panel                                      */
/*********************************************************************/
      /* Line 1 */
      strncpy(pnlmsg, "3705 Front Panel", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buf, sizeof(buf)-1,"\xF5\xC3\x11\x40\x5D\x1D\xF0\x28\x42\xF2%s\x1D\xF0",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl));

      /* Line 2 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC1\xD1\x28\x42\xF4\x28\x41\xF2\x3C\xC1\xF6\x40") + len;
      strcat(buf,buft);
      strncpy(pnlmsg, "Display A", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC1\x5F\x28\x42\xF4%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      len = snprintf (buft, sizeof(buft)-1, "\x11\xC1\xF9\x28\x42\xF4\x28\x41\xF2\x3C\xC2\x5E\x40") + len;
      strcat(buf,buft);
      strncpy(pnlmsg, "Display B", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC2\xC7\x28\x42\xF4%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* DISPLAY A, BYTE 0 */

      strncpy(pnlmsg, "+-----BYTE X------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC3\xF1\x28\x41\xF0\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC5\xC1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC6\xD1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC7\x61\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| BYTE X....... 7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC8\xF1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "+-----BYTE 0------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4A\xC1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| BYTE 0....... 0 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4B\xD1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| BYTE 1....... 1 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4C\x61\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| INDATA....... 2 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4D\xF1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| SAR.......... 3 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4F\xC1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| SDR.......... 4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x50\xD1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| OP REG....... 5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD1\x61\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| CLOCK........ 6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD2\xF1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| LEVI PROG.... 7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD4\xC1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "+----CC CHECKS----+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD5\xD1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* DISPLAY A, BYTE 1 */

      strncpy(pnlmsg, "-----------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC4\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC5\xD4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC9\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "-----BYTE 1------|", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC4\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "               0 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC5\xD4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "               1 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC6\xE4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " CS CYCLE..... 2 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC7\xF4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " I CYCLE...... 3 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC9\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "            +- 4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4A\xD4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " CYCLE TIME-|    |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4B\xE4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "            +- 5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4C\xF4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "            +- 6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4E\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " CLOCK TIME-|    |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4F\xD4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "            +- 7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x50\xE4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD1\xF4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD3\xC4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD4\xD4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "-----------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD5\xE4\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

       /* DISPLAY B, BYTE X */

      strncpy(pnlmsg, "+-----BYTE X------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC4\xD9\x28\x41\xF0\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);


      strncpy(pnlmsg, "|               4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC5\xE9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC6\xF9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC8\xC9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC9\xD9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* DISPLAY B, BYTE 0 */
      strncpy(pnlmsg, "+-----BYTE 0------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4A\xE9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| ADDR COMPARE  0 +", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4B\xF9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|               1 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4D\xC9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| IPL PHASE +-- 2 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4E\xD9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "|           +-- 3 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4F\xE9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| ADAPTER CHECK 4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x50\xF9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| I/O CHECK.... 5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD2\xC9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| ADDR EXCEPT.. 6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD3\xD9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| PROTECT CHECK 7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD4\xE9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "+-----------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD5\xF9\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);



      /* DISPLAY B, BYTE 1 */

      strncpy(pnlmsg, "-----BYTE 1------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC4\x6C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " INVALID OP... 0 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC5\x7C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "               1 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC7\x4C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "--ACTIVE LEVEL---+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC8\x5C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " C............ 2 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xC9\x6C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " Z............ 3 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4A\x7C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "-INTERRUPT LEVEL-+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4C\x4C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " PROG LEV 1... 4 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4D\x5C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " PROG LEV 2... 5 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4E\x6C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " PROG LEV 3... 6 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x4F\x7C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, " PROG LEV 4... 7 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD1\x4C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD2\x5C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD3\x6C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "                 |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD4\x7C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);


      strncpy(pnlmsg, "-----------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD6\x4C\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* Channel Adapters */

      strncpy(pnlmsg, "+-----Channel Adapter 1-----+"
                      "-----Channel Adapter 2-----+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\xD7\xF1\x28\x41\xF0\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "| A:           B:           |"
                       " A:           B:           |", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft),"\x11\xD9\xC1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      strncpy(pnlmsg, "+---------------------------+"
                      "---------------------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x5A\xD1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* PF KEYS */

      strncpy(pnlmsg, "PF1=CA1 A/B  PF2=CA2 A/B  PF3=LAMP TEST  "
                      "PF4=CC CHECK RESET", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1,"\x11\x5C\xF1\%s",
      prt_host_to_guest( pnlmsg,  (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf,buft);

      /* check channel adapter states */

      if (iob1->abswitch == 0)
      {
         strcat(buf,ca1b);
         len = len + sizeof(ca1b);
         strcat(buf,disabled);
         len = len + sizeof(disabled);
         strcat(buf,ca1a);
         len = len + sizeof(ca1a);
         if (iob1->bus_socket[0] > 0) {
            strcat(buf,active);
            len = len + sizeof(active);
           } else {
                strcat(buf,enabled);
                len = len + sizeof(enabled);
           }
       } else {
            strcat(buf,ca1a);
            len = len + sizeof(ca1a);
            strcat(buf,disabled);
            len = len + sizeof(disabled);
            strcat(buf,ca1b);
            len = len + sizeof(ca1b);
            if (iob1->bus_socket[1] > 0) {
            strcat(buf,active);
            len = len + sizeof(active);
           } else {
                strcat(buf,enabled);
                len = len + sizeof(enabled);
           }
       }
             if (iob2->abswitch == 0)     /* If CA2 A/B switch set to A */
      {
         /* Set CA2 B to Disabled */
         strcat(buf,ca2b);
         len = len + sizeof(ca2b);
         strcat(buf,disabled);
         len = len + sizeof(disabled);
         /* Now set CA2 A to actove or enabled */
         strcat(buf,ca2a);
         len = len + sizeof(ca2a);
         if (iob1->bus_socket[0] > 0) {
            strcat(buf,active);
            len = len + sizeof(active);
           } else {
                strcat(buf,enabled);
                len = len + sizeof(enabled);
           }
       } else {
             /* Set CA2 A to disabled */
            strcat(buf,ca2a);
            len = len + sizeof(ca2a);
            strcat(buf,disabled);
            len = len + sizeof(disabled);
            /* Now set CA2 B to either active or enabled */
            strcat(buf,ca2b);
            len = len + sizeof(ca2b);
            if (iob1->bus_socket[1] > 0) {
            strcat(buf,active);
            len = len + sizeof(active);
           } else {
                strcat(buf,enabled);
                len = len + sizeof(enabled);
           }
       }


      /* Decode display registers */
      strcat(buf,dbb0b0);
      if (Eregs_Out[0x72] & 0x8000)
         strcat(buf,revred);
      else
         strcat(buf,defpos);
         len = len + 9;                   /* address + attributes */

      strcat(buf,dbb0b1);
      if (Eregs_Out[0x72] & 0x4000)
         strcat(buf,revred);
      else
         strcat(buf,revred);
         len = len + 9;                   /* address + attributes */

      strcat(buf,dbb0b2);
      if (Eregs_Out[0x72] & 0x2000)
         strcat(buf,revred);
      else
         strcat(buf,defpos);
         strcat(buf,"\xF2");
         len = len + 10;                   /* address + attributes */

      strcat(buf,dbb0b3);
      if (Eregs_Out[0x72] & 0x1000)
         strcat(buf,revred);
      else
         strcat(buf,defpos);
         len = len + 9;                   /* address + attributes */




      /* End Markers */

      buf[len++] = IAC;
      buf[len++] = EOR_MARK;

      rc = send_packet (rpfd, (uint8_t *) buf,len, "Initial panel display");


    if (rc < 0) {
        printf("\nsend to client failed");
        return NULL;
    } /* end if(rc) */
        fcntl(rpfd, F_SETFL, fcntl(rpfd, F_GETFL,0) | O_NONBLOCK);

}
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

char* buf3270 (int row, int col)
{
   char trantab[] = {
        "\x40\xC1\xC2\xC3\xC4\xC5\xC6\xC7\xC8\xC9\x4A\x4B\x4C\x4D\x4E\x4F"
        "\x50\xD1\xD2\xD3\xD4\xD5\xD6\xD7\xD8\xD9\x5A\x5B\x5C\x5D\x5E\x5F"
        "\x60\x61\xE2\xE3\xE4\xE5\xE6\xE7\xE8\xE9\x6A\x6B\x6C\x6D\x6E\x6F"
        "\xF0\xF1\xF2\xF3\xF4\xF5\xF6\xF7\xF8\xF9\x7A\x7B\x7C\x7D\x7E\x7F"
   };
   uint16_t bfa;
   uint8_t  hrow;
   uint8_t  hcol;
   bfa = ((row-1)*80) + (col-1);
   hcol = bfa & 0x003F;
   hrow =  (bfa & 0x0FC0) >> 6;
   bfa = (trantab[hrow] << 8) + trantab[hcol];
   rowcol[0] = bfa >>8;
   rowcol[1] = bfa & 0x00FF;
   printf("\r decoded 3270 BFA %02X, %02X\n",rowcol[0],rowcol[1]);
   return rowcol;

}
