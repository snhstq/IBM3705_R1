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

   This module emulates several founctiopns of the 3705 front panel.
   To access the panel connect access port 37050 with a TN3270 emulator

   This module includes an interval timer that tiggers every 100msec a L3 interrupt.

 ********************************************************************************
*/

#include <stdio.h>
#include <sched.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include "i3705_defs.h"
#include "i3705_Eregs.h"               /* Exernal regs defs */

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

int rc, inp, i;
int32 hex_sw, rot_sw;

#define IAC      255                   /* Interpret as Command */
#define EOR_MARK 239                   /* End of record marker */
#define MAXPORTS 4

int rpfd;
int lpfd;
int sockopt;                           /* Used for setsocket option */
int rc, len;                           /* Return code from various rtns */
char wrkbyte[1];                       /* Work byte */
char bufc[6];
uint8_t bufh[5];
uint8_t mbyte;
uint16_t freebuf;
uint32_t maddr;

char *ipaddr;
char buf[8192], ibuf[256];
char buft[256];
char defpos[9] = {"\x29\x03\xC0\xF0\x41\xF0\x42\xF4\x40"};
char revred[9] = {"\x29\x03\xC0\xF0\x41\xF2\x42\xF2\x40"};
char revgrn[9] = {"\x29\x03\xC0\xF0\x41\xF2\x42\xF4\x40"};

char dbb0b4[3] = {"\x11\xC6\xE2"};
char dbb0b5[3] = {"\x11\xC7\xF2"};
char dbb0b6[3] = {"\x11\xC9\xC2"};
char dbb0b7[3] = {"\x11\x4A\xD2"};
char dbb1b0[3] = {"\x11\x4B\xE2"};
char active[21]   = {"\x29\x03\xC0\xF0\x41\xF2\x42\xF4\xC1\xC3\xE3\xC9\xE5\xC5\x29\x01\x41\xF0\x40\x40\0"};
char enabled[17]  = {"\x29\x03\xC0\xF0\x41\xF0\x42\xF1\xC5\xD5\xC1\xC2\xD3\xC5\xC4\x40\0"};
char disabled[17] = {"\x29\x03\xC0\xF0\x41\xF0\x42\xF2\xC4\xC9\xE2\xC1\xC2\xD3\xC5\xC4\0"};
char ca1a[4] = {"\x11\xD4\xC5\0"};
char ca1b[4] = {"\x11\xD5\xD5\0"};
char ca2a[4] = {"\x11\xD7\xF5\0"};
char ca2b[4] = {"\x11\xD9\xC5\0"};

char nibble[3] = {"\0\0\0"};
char hexsw[2] = {"\xF0\0"};
char hexswpos[16] = {"\x4D\xC1\xF0\x4D\xC4\xF0\x4D\xC7\xF0\x4D\x4A\xF0\x4D\x4D\xF0\0"};

char fs[10][12] = {{"\x11\xD3\x5B\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD4\x6B\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD5\x7B\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD7\x4B\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD8\x5B\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD8\xC9\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD6\xF9\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD5\xE9\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD4\xD9\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"},
                  {"\x11\xD3\xC9\x29\x03\xC0\xF0\x42\xF4\x41\xF2\0"}};

extern uint8 M[MAXMEMSIZE];
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
   uint8_t buffer[1024];               // Data buffer of 1K
   uint32_t bufferl;                   // Received data length
   struct sockaddr_in address[2];
   } *iob1, *iob2;

uint8_t  class;                        /* D=3270, P=3287, K=3215/1052 */
uint8_t  model;                        /* 3270 model (2, 3, 4, 5, X)  */
uint8_t  extended;                     /* Extended attributes (Y, N) */
uint16_t devnum;                       /* Requested device number */
char     group[16];                    /* Console group */
int row, col, pnlmsgl, dsswitch;
uint16_t bfa;
unsigned char rowcol[2];
char     pnlmsg[256];                  /* panel message */

extern int reg_bit(int reg, int bit_mask);
extern void wait();
extern int send_packet (int rpfd, uint8_t *buf, int len, char *caption);
extern int recv_packet (int rpfd, uint8_t *ibuf, int len, uint8_t delim);
extern int negotiate(int rpfd, uint8_t *class, uint8_t *model, uint8_t *extatr, uint16_t *devn, char *group);
extern uint8_t prt_host_to_guest( uint8_t *pnlmsgi,  uint8_t *pnlmsgo, const uint ilength  );
char* buf3270 (int row, int col);
char ebc2hex (char ebc0, char ebc1);


// **********************************************************
// The panel adaptor handling thread starts here...
// **********************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */

void *PNL_thread(void *arg) {
   fprintf(stderr, "PNL: Thread %d started succesfully... \n\r", syscall(SYS_gettid));

   // We can set one or more bits here, each one representing a single CPU
   cpu_set_t cpuset;

   // Select the CPU core we want to use
   int cpu = 2;

   CPU_ZERO(&cpuset);                  // Clears the cpuset
   CPU_SET( cpu , &cpuset);            // Set CPU on cpuset

   /*
    * cpu affinity for the calling thread
    * first parameter is the pid, 0 = calling thread
    * second parameter is the size of your cpuset
    * third param is the cpuset in which your thread
    * will be placed. Each bit represents a CPU.
    */
   sched_setaffinity(0, sizeof(cpuset), &cpuset);


   signal (SIGALRM, sig_handler);      /* Interval timer */
   timer_msec(100);                    // <=== sets the 3705 interval timer

   int disp_regA, disp_regB;
   int inp_h, inp_l;
   int five_lt = 0x00;

   struct sockaddr_in  sin, *sin2;     /* bind socket address structure     */
   struct ifaddrs *nwaddr, *ifa;       /* interface address structure       */

   getifaddrs(&nwaddr);                /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("PNL: Using network Address %s on %s for 3270 panel connections\n\r", ipaddr, ifa->ifa_name);
   rpfd = 0;
   lpfd = socket(AF_INET, SOCK_STREAM, 0);


   /* Reuse the address regardless of any */
   /* spurious connection on that port    */
   sockopt = 1;
   setsockopt(lpfd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));

   /* Bind the socket */
   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr = inet_addr(ipaddr);
   sin.sin_port=htons(37050);
   rc=bind(lpfd, (struct sockaddr *)&sin, sizeof(sin));
   if (rc < 0) {
      printf("PNL: Socket bind failed\n\r");
      return NULL;
   }
   rpfd = 0;
   /* Start the listen */
   rc = listen(lpfd, 1);
   if (rc < 0) {
      //WRMSG(HHC01000, "E", SSID_TO_LCSS(ca->dev->ssid), devnum, "listen()", strerror(HSO_errno));
      return NULL;
   }

/********************************************************************/
/* Until there is a 3270 connection, nothing happens                */
/********************************************************************/

   if (rpfd < 1) {
      printf("PNL: Waiting for 3270 connection \n\r");
      rpfd = accept(lpfd, NULL, 0);

      /* Negotiate telnet parameters */
      rc = negotiate (rpfd, &class, &model, &extended, &devnum, group);
      if (rc != 0) {
         printf("PNL: 3270 negotiation failed; rc = %d\n\r", rc);
         close (rpfd);
         rpfd = 0;
         // if (clientip) free(clientip);
         return 0;
      }
      else printf("PNL: 3270 negotiation OK\n\r");

/*********************************************************************/
/* Build the initial 3705 panel screen                               */
/*********************************************************************/

      /* Line 1 */
      strncpy(pnlmsg, "3705 Front Panel", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buf, sizeof(buf)-1, "\xF5\xC3\x11\x40\x5D\x1D\xF0\x29\x02\xC0\xF0\x42\xF2%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl));

      /* Line 2 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC1\x50\x1D\xF0\x29\x02\xC0\xF0\x42\xF4\x3C\xC2\x5F\x6D\x1D\xF0") + len;
      strcat(buf, buft);

      /* Status and Error indicators */
      /* line 4 */
      strncpy(pnlmsg, "IPL PHASE:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC4\x6A\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 5 */
      strncpy(pnlmsg, "+----CCU CHECKS-----+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC5\x40\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "+--DISPLAY A--+--DISPLAY B--+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC5\xD8\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "FREE BUFFERS:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC5\x7A\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 6 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x50\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "ADAPTER CHECK", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xD2\x29\x02\xC0\xF0\x42\xF1\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xE4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Display A */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xE8\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6A\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6C\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6E\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x40\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF1\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF3\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x40\x1D\xF0") + len;
      strcat(buf, buft);

      /* Display B */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF6\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF8\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x7A\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x7C\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x40\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x7F\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xC1\x29\x03\xC0\xF0\x42\xF6\x41\xF2\x40\x40\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xC4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 7 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\x60\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "I/O CHECK", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xE2\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xF4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      strncpy(pnlmsg, "+-X----0----1-+-X----0----1-+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xF8\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      /* Line 8 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC8\xF0\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "ADDRESS EXCEPT", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xC8\xF2\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x49\xC4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 9 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4A\x40\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "PROTECT CHECK", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4A\xC2\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4A\xD4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 10 */
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4B\x50\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "INVALID OP", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4B\xD2\x29\x02\xC0\xF0\x42\xF1\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4B\xE4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);
      strncpy(pnlmsg, " + A  B  C  D  E +", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4B\x6D\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 11 */
      strncpy(pnlmsg, "+-------------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4C\x60\x29\x02\xC0\xF0\x42\xF4\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      /* Hex Switches */
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\x40\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\xC3\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\xC6\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\xC9\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\x4C\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
      strcat(buf, buft);

      /* Channel Adapters */
      /* Line 16 */
      strncpy(pnlmsg, "+-Channel Adapter 1-+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD2\xF0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 17 */
      strncpy(pnlmsg, "| A:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft), "\x11\xD4\xC0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD4\xD4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 18 */
      strncpy(pnlmsg, "| B:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft), "\x11\xD5\xD0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD5\xE4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 19 */
      strncpy(pnlmsg, "+-Channel Adapter 2-+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD6\x60\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 20 */
      strncpy(pnlmsg, "| A:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft), "\x11\xD7\xF0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD8\xC4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 21 */
      strncpy(pnlmsg, "| B:", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft), "\x11\xD9\xC0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD9\xD4\x29\x02\xC0\xF0\x42\xF4\x6A") + len;
      strcat(buf, buft);

      /* Line 22 */
      strncpy(pnlmsg, "+-------------------+", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x5A\xD0\x29\x02\xC0\xF0\x42\xF4\%s",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Display / Function Select */
      /* Line 14 */
      strncpy(pnlmsg, "DISPLAY/FUNCTION SELECT", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x50\x6B\x29\x03\xC0\xF0\x42\xF1\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 16 */
      strncpy(pnlmsg, "TAR&OP REGISTER", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD3\xC9\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "STATUS", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD3\x5B\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 17 */
      strncpy(pnlmsg, "FUNCTION 6", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD4\xD9\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "STORAGE ADDRESS", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD4\x6B\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 18 */
      strncpy(pnlmsg, "FUNCTION 5", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD5\xE9\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "REGISTER ADDRESS", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD5\x7B\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 19 */
      strncpy(pnlmsg, "FUNCTION 4", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD6\xF9\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "FUNCTION 1", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD7\x4B\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 20 */
      strncpy(pnlmsg, "FUNCTION 3", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD8\xC9\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);
      strncpy(pnlmsg, "FUNCTION 2", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\xD8\x5B\x29\x03\xC0\xF0\x42\xF4\x41\xF0\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 23 PF KEYS */
      strncpy(pnlmsg, "PF1=CA1 A/B  PF2=CA2 A/B  PF3=D/F SELECT  "
                      "PF4=CHECK RESET PF5=SET HEX SWITCH", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x5B\x60\x29\x02\xC0\xF0\x42\xF1\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);

      /* Line 24 PF KEYS */
      strncpy(pnlmsg, "ENTER=SET ADDRESS DISPLAY ", sizeof(pnlmsg));
      pnlmsgl = strlen(pnlmsg);
      len = snprintf (buft, sizeof(buft)-1, "\x11\x5C\xF0\x29\x02\xC0\xF0\x42\xF1\%s\x1D\xF0",
      prt_host_to_guest( pnlmsg, (uint8_t*) pnlmsg,  pnlmsgl)) + len;
      strcat(buf, buft);


      strcat(buf, "\x11\x4D\x41\x13");
      len = len + 4;

      /* End Markers */

      buf[len++] = IAC;
      buf[len++] = EOR_MARK;

      rc = send_packet (rpfd, (uint8_t *) buf, len, "Initial panel display");

      if (rc < 0) {
         printf("PNL: send to panel failed\n\r");
         return NULL;
      } /* end if(rc) */

      // fcntl(rpfd, F_SETFL, fcntl(rpfd, F_GETFL, 0) | O_NONBLOCK);

   }
      /* set switch in function 0 */
      dsswitch = 0;
      Eregs_Inp[0x72] |= 0x0002;
      len = 0;
      buf[0] = '\0';
      strcat(buf, "\xF1\xC3");
      len = len + 2;
      strcat(buf, fs[dsswitch]);
      len = len + 11;

      /* End Markers */

      buf[len++] = IAC;
      buf[len++] = EOR_MARK;

      rc = send_packet (rpfd, (uint8_t *) buf, len, "Initial F/S Switch");

   // *******************************************************
   // Start port scanning for input and update outputs
   // *******************************************************
while (1) {
   rc = read(rpfd, ibuf, sizeof(ibuf));
   if (rc > 0) {
      printf("PNL: received %d bytes from panel\n\r", rc);
      printf("PNL: received %02x, %02X, %02X, %02X, %02X\n\r", ibuf[0], ibuf[1], ibuf[2], ibuf[3], ibuf[4]);
   len = 0;
   buf[0] = '\0';
   strcat(buf, "\xF1\x03");
   len = len + 2;

   switch (ibuf[0]) {
      case 0xF1:
         if (iob1->abswitch == 0) iob1->abswitch = 1;
             else iob1->abswitch = 0;
         break;

      case 0xF2:
         if (iob2->abswitch == 0) iob2->abswitch = 1;
            else iob2->abswitch = 0;
         break;

      case 0xF3:
         /* The display / function switch will step in a clockwise direction */
         /* reset highlighting current switch setting */
         strcat(buf, fs[dsswitch]);
         len = len + 11;
         buf[len-1] &= 0xF0;           /* extended highlighting default */
         /* Point to next switch setting */
         dsswitch = dsswitch + 1;
         if (dsswitch > 9) dsswitch = 0;
         strcat(buf, fs[dsswitch]);
         len = len + 11;

         /* set the Display/Function select switch controls register */
         /* Not the most intelligent approach though... */
         switch (dsswitch) {
            case 0:
               /* Display status from Display A en Display B */
               Eregs_Inp[0x72] &= 0x0002;  /* Reset previous switch setting */
               /* Decode display registers */
               strncat(buf, dbb0b4, sizeof(dbb0b4));
               if (Eregs_Out[0x72] & 0x0800)
                  strncat(buf, revred, sizeof(revred));
               else
                  strncat(buf, defpos, sizeof(defpos));
               len = len + sizeof(dbb1b0) + sizeof(defpos);  /* address + attributes */

               strncat(buf, dbb0b5, sizeof(dbb0b5));
               if (Eregs_Out[0x72] & 0x0400)
                  strncat(buf, revred, sizeof(revred));
               else
                  strncat(buf, defpos, sizeof(defpos));
               len = len + sizeof(dbb1b0) + sizeof(defpos);  /* address + attributes */

               strncat(buf, dbb0b6, sizeof(dbb0b6));
               if (Eregs_Out[0x72] & 0x0200)
                  strncat(buf, revred, sizeof(revred));
               else
                  strncat(buf, defpos, sizeof(defpos));
               len = len + sizeof(dbb1b0) + sizeof(defpos);  /* address + attributes */

               strncat(buf, dbb0b7, sizeof(dbb0b7));
               if (Eregs_Out[0x72] & 0x0100)
                  strncat(buf, revred, sizeof(revred));
               else
                  strncat(buf, defpos, sizeof(defpos));
               len = len + sizeof(dbb1b0) + sizeof(defpos);  /* address + attributes */

               strncat(buf, dbb1b0, sizeof(dbb1b0));
               if (Eregs_Out[0x72] & 0x0080)
                  strncat(buf, revred, sizeof(revred));
               else
                  strncat(buf, defpos, sizeof(defpos));
               len = len + sizeof(dbb1b0) + sizeof(defpos);  /* address + attributes */
            break;
            case 1:
               /* Set Hex Switches A, C and E to their default */
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\x40\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\xC6\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\x4C\x29\x03\xC0\xD0\x42\xF5\x41\xF4\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               Eregs_Inp[0x72] |= 0x1000;
            break;
            case 2:
               /* Hex Switches */
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\x40\x29\x03\xC0\xD0\x42\xF2\x41\xF2\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\xC6\x29\x03\xC0\xD0\x42\xF2\x41\xF2\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               len = snprintf (buft, sizeof(buft)-1,
                  "\x11\x4D\x4C\x29\x03\xC0\xD0\x42\xF2\x41\xF2\xF0\x1D\xF0") + len;
               strcat(buf, buft);
               /* Set Cursor to switch B position */
               len = snprintf (buft, sizeof(buft)-1, "\x11\x4D\xC3\x05\x13") + len;
               strcat(buf, buft);
               Eregs_Inp[0x72] &= ~0x1000;
               Eregs_Inp[0x72] |= 0x0800;
            break;
            case 3:
               Eregs_Inp[0x72] &= ~0x0800;
               Eregs_Inp[0x72] |= 0x0040;
            break;
            case 4:
               Eregs_Inp[0x72] &= ~0x0040;
               Eregs_Inp[0x72] |= 0x0020;
            break;
            case 5:
               Eregs_Inp[0x72] &= ~0x0020;
               Eregs_Inp[0x72] |= 0x0010;
            break;
            case 6:
               Eregs_Inp[0x72] &= ~0x0010;
               Eregs_Inp[0x72] |= 0x0008;
            break;
            case 7:
               Eregs_Inp[0x72] &= ~0x0008;
               Eregs_Inp[0x72] |= 0x0004;
            break;
            case 8:
               Eregs_Inp[0x72] &= ~0x0004;
               Eregs_Inp[0x72] |= 0x0002;
            break;
            case 9:
               Eregs_Inp[0x72] &= ~0x0002;
            break;
         }
         break;

         case 0xF5:
            /* First check if the cursor is on a switch position */
            for (i = 0; i < 15; i = i + 3) {
               if ((ibuf[1] == hexswpos[i]) && (ibuf[2] == hexswpos[i+1])) break;
            }
            if (hexswpos[i] == 0x00) break;  /* Cursor not on a switch */

            hexsw[0] = hexswpos[i+2] + 0x01;
            if (hexsw[0] == 0xFA) hexsw[0] = 0xC1;
            if (hexsw[0] == 0xC7) hexsw[0] = 0xF0;
            hexswpos[i+2] = hexsw[0];
            strcat(buf, hexsw);
            len = len + 1;
            /* Store hex switches in register 71 */
            /* Switch A excluded for now */
            /* Convert EBCDIC register in switches B and C to HEX value */
            Eregs_Inp[0x71] = (ebc2hex(hexswpos[5], hexswpos[8])) << 8;
            /* Convert EBCDIC register in switches D and E to HEX value */
            Eregs_Inp[0x71] = Eregs_Inp[0x71] + (ebc2hex(hexswpos[11], hexswpos[14]));

         break;
         case 0x7D:                    /* Enter = Set Address Disiplay */
            switch (dsswitch) {
               case 1:                 /* Display memory contents */
                  /* Display switches A-E in Display A */
                  /* Display byte X of memory address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6B") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[2], 1);
                  len = len + 1;
                  /* set high order nibble of byte 0 to high order nibble of memory address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6F") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[5], 1);
                  len = len + 1;
                  /* display low order nibble of byte 0  to low order nibble of memory address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF0") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[8], 1);
                  len = len + 1;
                  /* display high order nibble of byte 1 to high order nibble of memory address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF4") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[11], 1);
                  len = len + 1;
                  /* set low order nibble of byte 1 to low  order nibble of memory address  */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF5") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[14], 1);
                  len = len + 1;
                  /* Convert EBCDIC memory address in switches A and E to HEX value */
                  hexsw[0] = ebc2hex(0xF0, hexswpos[2]);           // Switch A
                  maddr = (hexsw[0] & 0x0F) << 16;
                  hexsw[0] = ebc2hex(hexswpos[5], hexswpos[8]);    // Switch B and C
                  maddr = maddr + (hexsw[0] << 8);
                  hexsw[0] = ebc2hex(hexswpos[11], hexswpos[14]);  // Switch D and E
                  maddr = maddr + hexsw[0];
                  /* Display memory contents in Display B */
                  /* Set Byte 0 to blank */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x7D\x40") + len;
                  strcat(buf, buft);
                  /* Byte 1 will show content of memory location */
                  mbyte = M[maddr];
                  nibble[0] = (mbyte >> 4) & 0x000F;
                  nibble[0] |= 0xF0;
                  if (nibble[0] > 0xF9) nibble[0] = nibble[0] - 0x39;
                  nibble[1] = (mbyte) & 0x000F;
                  nibble[1] |= 0xF0;
                  if (nibble[1] > 0xF9) nibble[1] = nibble[1] - 0x39;
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xC2") + len;
                  strcat(buf, buft);
                  strcat(buf, nibble);
                  len = len + 2;
               break;
               case 2:                 /* Display register contents  */
                  /* Display switch B and D in Display A */
                  /* set byte X to blank */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6B\x40") + len;
                  strcat(buf, buft);
                  /* set high order nibble of byte 0 to high order nibblle of register address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x6F") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[5], 1);
                  len = len + 1;
                  /* set low order nibble of byte 0 to blank */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF0\x40") + len;
                  strcat(buf, buft);
                  /* set high order nibble of byte 1 to high order nibblle of register address */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF4") + len;
                  strcat(buf, buft);
                  strncat(buf, &hexswpos[11], 1);
                  len = len + 1;
                  /* set low order nibble of byte 1 to blank */
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\xF5\x40") + len;
                  strcat(buf, buft);

                  /* Convert EBCDIC register in switches B and D to HEX value */
                  hexsw[0] = ebc2hex(hexswpos[5], hexswpos[11]);

                  /* Display register contents in Display B */
                  /* Byte 0 */
                  nibble[0] = (Eregs_Inp[hexsw[0]] >> 12) & 0x0F;
                  nibble[0] |= 0xF0;
                  if (nibble[0] > 0xF9) nibble[0] = nibble[0] - 0x39;
                  nibble[1] = (Eregs_Inp[hexsw[0]] >> 8) & 0x0F;
                  nibble[1] |= 0xF0;
                  if (nibble[1] > 0xF9) nibble[1] = nibble[1] - 0x39;
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC6\x7D") + len;
                  strcat(buf, buft);
                  strcat(buf, nibble);
                  len = len + 2;
                  /* Byte 1 */
                  nibble[0] = (Eregs_Inp[hexsw[0]] >> 4) & 0x000F;
                  nibble[0] |= 0xF0;
                  if (nibble[0] > 0xF9) nibble[0] = nibble[0] - 0x39;
                  nibble[1] = (Eregs_Inp[hexsw[0]]) & 0x000F;
                  nibble[1] |= 0xF0;
                  if (nibble[1] > 0xF9) nibble[1] = nibble[1] - 0x39;
                  len = snprintf (buft, sizeof(buft)-1, "\x11\xC7\xC2") + len;
                  strcat(buf, buft);
                  strcat(buf, nibble);
                  len = len + 2;

                  printf("Display 1: %02X %05X\n\r", hexsw[0], Eregs_Out[hexsw[0]]);
               break;
            default:
               break;
            }   // End of switch (dsswitch)
         break;
         }

         /* End Markers */
         buf[len++] = IAC;
         buf[len++] = EOR_MARK;

         rc = send_packet (rpfd, (uint8_t *) buf, len, "Execute ");
      } else {
         printf("PNL: receive from panel failed\n\r");
         return NULL;
      }  /* end if(rc) */

      Eregs_Inp[0x7F] |= 0x0200;
      inter_req_L3 = ON;               /* Panel L3 request flag */
      while (reg_bit(0x7F, 0x0200) == ON)
         wait();

      len = 0;
      buf[0] = '\0';
      strcat(buf, "\xF1\xC3");
      len = len + 2;

      printf("Display 1: %05X\n\r", Eregs_Out[0x71]);
      printf("Display 2: %05X\n\r", Eregs_Out[0x72]);

      /* Pick up free buffer count */
      freebuf = (M[0x0754] << 8) + M[0x0755];
      snprintf(bufc, 5, "%4d", freebuf);
      for (int i = 0; i < 4; i++)
         bufh[i] = (bufc[i] | 0xF0);
      strcat(buf, "\x11\xC6\xC8\x29\x02\xC0\xF0\x42\xF4");
      len = len + 9;
      strcat(buf, bufh);
      len = len + 4;

      /* IPL Phase update */
      wrkbyte[0] = (0xF0 | ((Eregs_Out[0x72] & 0x3000) >> 12));
      strcat(buf, "\x11\xC4\x7B\x29\x02\xC0\xF0\x42\xF4");
      len = len + 9;
      strcat(buf, wrkbyte);
      len = len + 1;

      /* check channel adapter states */
      /* Channel Adapter 1 */
      if (iob1->abswitch == 0) {
         len = snprintf (buft, sizeof(buft)-1, ca1b) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, disabled) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, ca1a) + len;
         strcat(buf, buft);
         if (iob1->bus_socket[0] > 0) {
            len = snprintf (buft, sizeof(buft)-1, active) + len;
            strcat(buf, buft);
         } else {
            len = snprintf (buft, sizeof(buft)-1, enabled) + len;
            strcat(buf, buft);
         }
      } else {
         len = snprintf (buft, sizeof(buft)-1, ca1a) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, disabled) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, ca1b) + len;
         strcat(buf, buft);
         if (iob1->bus_socket[1] > 0) {
            len = snprintf (buft, sizeof(buft)-1, active) + len;
            strcat(buf, buft);
         } else {
            len = snprintf (buft, sizeof(buft)-1, enabled) + len;
            strcat(buf, buft);
         }
      }

      /* Channel adapter 2 */
      if (iob2->abswitch == 0) {
         len = snprintf (buft, sizeof(buft)-1, ca2b) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, disabled) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, ca2a) + len;
         strcat(buf, buft);
         if (iob2->bus_socket[0] > 0) {
            len = snprintf (buft, sizeof(buft)-1, active) + len;
            strcat(buf, buft);
         } else {
            len = snprintf (buft, sizeof(buft)-1, enabled) + len;
            strcat(buf, buft);
         }
      } else {
         len = snprintf (buft, sizeof(buft)-1, ca2a) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, disabled) + len;
         strcat(buf, buft);
         len = snprintf (buft, sizeof(buft)-1, ca2b) + len;
         strcat(buf, buft);
         if (iob2->bus_socket[1] > 0) {
            len = snprintf (buft, sizeof(buft)-1, active) + len;
            strcat(buf, buft);
         } else {
            len = snprintf (buft, sizeof(buft)-1, enabled) + len;
            strcat(buf, buft);
         }
      }

      /* End Markers */
      buf[len++] = IAC;
      buf[len++] = EOR_MARK;

      rc = send_packet (rpfd, (uint8_t *) buf, len, "Channel status update");
      if (rc < 0) {
         printf("\nPNL: send to client failed");
         return NULL;
      } /* end if(rc) */
      usleep(1000);
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
   if (test_mode == OFF) {
      Eregs_Inp[0x7F] |= 0x0004;
      timer_req_L3 = ON;
   //   while (reg_bit(0x77, 0x0040) == OFF)
   //   wait();
   }
}

char ebc2hex (char ebc0, char ebc1)
{
   char hexbyte;
   /* Convert  EBCDIC values register to HEX value */
   /* first do high order nibble */
   hexbyte = ebc0;
   if (hexbyte < 0xF0) hexbyte = hexbyte + 0x39;   /* if value between xC1-xC6 (A-F) convert to xFA-xFF */
   hexbyte = hexbyte << 4;                         /* shift to high order nibble */
   /* Do low order nibble */
   if (ebc1 < 0xF0) hexbyte = hexbyte + ((ebc1 + 0x39) & 0x0F); /* if value between xC1-xC6 (A-F) convert to xFA-xFF */
      else
      hexbyte = hexbyte + (ebc1  & 0x0F);  /* else simply add the low order nibble */
   return hexbyte;
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
   hrow = (bfa & 0x0FC0) >> 6;
   bfa = (trantab[hrow] << 8) + trantab[hcol];
   rowcol[0] = bfa >>8;
   rowcol[1] = bfa & 0x00FF;
   printf("\r decoded 3270 BFA %02X, %02X\n", rowcol[0], rowcol[1]);
   return rowcol;

}
