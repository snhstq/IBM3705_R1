/* 3705_chan_T1.c IBM 3705 Channel Adaptor Type 1 simulator

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


   *** CA1 input (CCU output) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CAISC      0x60         // CA Initial selection control.
   CAISD      0x61         // CA Initial selection addr and command.
   CASSC      0x62         // Data/status control.
   CASSA      0x63         // CA Address and ESC status.
   CASD12     0x64         // Data buffer bytes 1 and 2.
   CAS0D      0x65         // Data buffer bytes 3 and 4.
   CARNSTAT   0x66         // CA NSC status byte.
   CAECR      0x67         // CA Controls.

   *** CA1 output (CCU input) Eregs ***
   Label      Ereg         Function
   --------------------------------------------------------------
   CAISC      0x60         // CA Reset initial selection.
              0x61         // Unused.
              0x62         // Unused.
   CASSC      0x62         // Data/status control
   CASSA      0x63         // CA Address and ESC status.
   CASD12     0x64         // Data buffer bytes 1 and 2.
   CASD34     0x65         // Data buffer bytes 3 and 4.
   CARNSTAT   0x66         // CA NSC status byte.
   CAECR      0x67         // CA Controls.

*/

#include "sim_defs.h"
#include "i3705_defs.h"
#include "i3705_Eregs.h"     /* Exernal regs defs */
#include <signal.h>
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <stdlib.h>
#include <sys/socket.h>

#define IMAX 4096            // Input buffer size Random. Need to define a more rational value
#define RMAX 64              // Response buffer size Random. Need to define a more rational value

#define PORT 3704            // TCP/IP port number.
#define SA struct sockaddr_in
#define TRUE  1
#define FALSE 0

// CSW Unit Status conditions.  Channel status conditions not defined (yet).
#define CSW_ATTN 0x80        // Attention
#define CSW_SMOD 0x40        // Status Modifier
#define CSW_UEND 0x20        // Control Unit End
#define CSW_BUSY 0x10        // Busy
#define CSW_CEND 0x08        // Channel End (CE)
#define CSW_DEND 0x04        // Device End (DE)
#define CSW_UCHK 0x02        // Unit check
#define CSW_UEXC 0x01        // Unit Exception

#define MAXHOSTS 4

extern int32 Eregs_Inp[];
extern int32 Eregs_Out[];
extern int8  CA1_DS_req_L3;  /* Chan Adap Data/Status request flag */
extern int8  CA1_IS_req_L3;  /* Chan Adap Initial/Sel request flag */
char data_buffer[IMAX];
char response_buffer[RMAX];
int i;
uint8_t nobytes, tcount;

// Declaration of thread condition variable
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

// Declaring mutex
//pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
 pthread_mutex_t lock;

int reg_bit(int reg, int bit_mask);
int Ireg_bit(int reg, int bit_mask);
void wait();

struct CCW   /* Channel Command Word */
{
   uint8_t  code;
   uint8_t  data_address[3];
   uint8_t  flags;
   uint8_t  chain;           // Format 0. Chain is not correct (yet), but aligned to Hercules
   uint16_t count;
} ccw;

struct CSW   /* Channel Status Word */
{
   uint8_t  key;
   uint8_t  data_address[3];
   uint8_t  unit_conditions;
   uint8_t  channel_conditions;
   uint16_t count;
} csw;

struct IO3705 {
  int socket_3705;
    int max_sd;
    int addrlen;
    int ccw_socket;
    int tag_socket;
    int activity;
    int valred;
    int rcvid;
    uint16_t devnum;
    uint8_t buffer[4096];   // Data buffer of 4K
    uint32_t bufferl;       // Received data length
  struct sockaddr_in address;
  } iob;

int wrkid, offset;

// ************************************************************
// Function to format and display incomming data from host
// ************************************************************
void print_hex(char *buffptr,int buf_len)
{
   printf("\nRecord length: %X  (hex)\n\r", buf_len);
      for (int i = 0; i < buf_len; i++) {
         printf("%02X ",(unsigned char)buffptr[i]);
         if ((i + 1) % 16 == 0)
            printf("\n\r");
      }
      printf("\n\r");
   return;
}

// ************************************************************
// Function to wait for TCP connection from host
// ************************************************************
int host_connect(struct IO3705 *iobptr, int *sockptr)
{
   // Listen and verify
   if ((listen(iobptr->socket_3705, 5)) != 0) {
      printf("\nCA1: Listen failed...\n\r");
      exit(0);
   }
   else
      printf("\nCA1: RPi 3705 waiting for host...\n\r");

   // Accept the incoming connection
   iobptr->addrlen = sizeof(iobptr->address);

   *sockptr = accept(iobptr->socket_3705, (struct sockaddr *)&iobptr->address, (socklen_t*)&iobptr->addrlen);
   if (iobptr->ccw_socket < 0) {
      printf("\nCA1: Host accept failed...\n\r");
      return -1;
   }
   else
      printf("\nCA1: New connection, socket fd is %d, ip is : %s, port : %d \n\r",
               *sockptr, inet_ntoa(iobptr->address.sin_addr), ntohs
               (iobptr->address.sin_port));
   return 0;
}

// ************************************************************
// Function to send CA return status to the host
// ************************************************************
void send_carnstat(int sockptr, char *carnstat, char *ackbuf)
{
   int rc;                      /* Return code */

         // Send CA retun status to host
      *carnstat = 0x00;
      *carnstat = (Eregs_Out[0x66] & 0x00FF);
      printf("CARNSTAT: %04X via socket %d\n\r", Eregs_Out[0x66], sockptr);
      rc = send(sockptr, carnstat, 1, 0);
      printf("CA1: Send %d bytes on socket %d\n\r", rc, sockptr);
   if (rc < 0) {
      printf("\nCA1: CA status send to host failed...\n\r");
      return;
      }
      // Wait for the ACK from the host
      rc = recv(sockptr, ackbuf, 1,0);
      printf("CA1: Ack received %02X on socket %d\n\r",*ackbuf,sockptr);
//      Eregs_Out[0x66] &= ~0xFFFF;                      // Reset CA status bytes
      Eregs_Out[0x66] = 0x0000;                      // Reset CA status bytes

   return;
}  /* end function send_carnstat */

// ************************************************************
// Function to wait for an ACK from the host
// ************************************************************
void recv_ack(int sockptr)
{
   char ackbuf;
   int rc;
   rc = read(sockptr, &ackbuf, 1);
   return;
}

// ************************************************************
// Function to send an ACK to the host
// ************************************************************
void send_ack(int sockptr)
{
   char ackbuf;
   int rc;
   rc = send(sockptr, &ackbuf, 1, 0);
   return;
}

// ************************************************************
// Function to read data from TCP socket
// ************************************************************
int read_socket(int sockptr, char *buffptr, int buffsize)
{
   int reclen;
   int n;
   bzero(buffptr, buffsize);
   reclen = read(sockptr, buffptr, buffsize);
   return reclen;
}

// ************************************************************
// Function to send response data to the host
// ************************************************************
int send_socket(int sockptr, char *respp, int respsize)
{
   int rc;                      /* Return code */
   rc = send (sockptr, respp, respsize, 0);

   if (rc < 0) {
      printf("\nCA1: Send to host failed...\n\r");
      return -1;
   }
   return 0;
}  /* end function send_SIMH */

// ************************************************************
// Function to create and open TCP socket
// ************************************************************
int open_socket(int *sockfd)
{
   struct sockaddr_in servaddr, cli;
   // SIMH socket creation
   *sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if (*sockfd == -1) {
      printf("\nCA1: TCP socket creation failed...\n\r");
      return(-1);
   }
   else
      printf("\nCA1: TCP socket successfully created...\n\r");
   bzero(&servaddr, sizeof(servaddr));

   // Assign IP addr and PORT number
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
   servaddr.sin_port = htons(3706);

   // Connect the SIMH socket to host socket
   while (connect(*sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
      printf("\nCA1: SIMH --> HOST connection failed! Retrying in 10 sec...\n\r");
      sleep(10);
   }
   printf("\nCA1: SIMH succesfully connected with HOST...\n\r");
   return(0);
}
// ************************************************************
// Thread for sending Attention interrupts to the host
// ************************************************************

void *CA1_ATTN(void *arg) {
  struct IO3705 *iob;
  iob = arg;
  int rc;
  char carnstat, ackbuf;
  ackbuf = 0x00;

  while (1) {
     if (((Eregs_Out[0x67] & 0x0040) == 0x0040) || ackbuf == 0xF8 ) {
        // Grab the lock to avoid sync issues
        pthread_mutex_lock(&lock);
        printf("CA1: L3 register 67 %04X \n\r", Eregs_Out[0x67]);
        Eregs_Inp[0x62] |= 0x0100;               // Set Program requested L3 interrupt
        Eregs_Inp[0x77] |= 0x0010;               // Set L3 Data Service Request
        CA1_DS_req_L3 = ON;                      // Chan Adap Data Service  request flag
        while (Ireg_bit(0x77, 0x0010) == ON) wait();
        Eregs_Out[0x67] &= ~0x0040;              // Reset L3 DS/ request
        printf("CA1: Sending Return status\n\r");
        // Send CA retun status to host
        send_carnstat(iob->tag_socket,&carnstat, &ackbuf);
        // Release the lock
        pthread_mutex_unlock(&lock);
     }
     usleep(10);
  }
return 0;
}

// ************************************************************
// The channel adaptor handling thread starts here...
// ************************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */
void *CA1_thread(void *arg) {
   int rc;
   int cc = 0;
   int sockfc = -1;
   pthread_t id;
   char carnstat, ackbuf;

   // Init the lock
   if (pthread_mutex_init(&lock, NULL) !=0)  {
      printf("CA1: Lock initialization failed \n\r");
      exit(EXIT_FAILURE);
   }

   // Create the 3705 socket and verify
   while (reg_bit(0x67, 0x0008) == OFF) wait();  // Wait for miniROS to request Chan enable

   if ((iob.socket_3705 = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
   }

   // Type of socket created
   iob.address.sin_family = AF_INET;
   iob.address.sin_addr.s_addr = INADDR_ANY;
   iob.address.sin_port = htons( PORT );

   // Set SIMH 3705 socket to allow multiple connections
   int flag = 1;
   if (-1 == setsockopt(iob.socket_3705, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA1: Setsockopt failed\n\r");
   }

   // Bind the socket to localhost port PORT
   if (bind(iob.socket_3705, (struct sockaddr *)&iob.address, sizeof(iob.address)) < 0) {
      perror("bind failed");
      exit(EXIT_FAILURE);
   }
   // Now server is ready to listen
   printf("CA1: TCP listener on port %d \n\r", PORT);

   // Wait for ccw connection from host
   rc = host_connect(&iob, &iob.ccw_socket);

   if (rc == 0) {
      // Get device number
      rc = read_socket( iob.ccw_socket, iob.buffer, sizeof(iob.buffer));
      iob.devnum = (iob.buffer[0] << 8) | iob.buffer[1];
      printf("\nCA1: Connected to device %04X\n\r", iob.devnum);
   }

   // Wait for tag connection from host
   rc = host_connect(&iob, &iob.tag_socket);

   if (rc == 0) {
      printf("CA1: Attention connection established with host on socket %d\n\r", iob.tag_socket);
      pthread_create(&id, NULL, CA1_ATTN, &iob);
      printf("CA1: Attention thread started succesfully... \n\r");
   }
   Eregs_Inp[0x77] &= ~0x0018;           // Reset inital sel and  data/serv lvl3 interrupt
   CA1_DS_req_L3 = OFF;                  // Chan Adap Data/Status request flag
   CA1_IS_req_L3 = OFF;                  // Chan Adap Initial Sel request flag
   Eregs_Inp[0x62] &= ~0x0400;           // Reset channel stop


   while(1) {
   // We do this for ever and ever...
   /**************************************************************/
   /*    Read channel command from host                          */
   /*                                                            */
   /*    This a a raw version: it assumes no pending operation   */
   /*    Channel status tests need to be added                   */
   /*                                                            */
   /**************************************************************/
      rc = read_socket( iob.ccw_socket, iob.buffer, sizeof(iob.buffer));
      if (rc == 0) {
         // Host disconnected, get details and print it
         getpeername(iob.ccw_socket, (struct sockaddr*)&iob.address, \
                (socklen_t*)&iob.addrlen);
         printf("\nHost disconnected, IP: %s, port: %d \n\r",
                inet_ntoa(iob.address.sin_addr), ntohs(iob.address.sin_port));

         // Close the socket and mark as 0 in list for reuse
         close(iob.ccw_socket);

         iob.ccw_socket = -1;
         // Wait for new connection from host
         rc = host_connect(&iob, &iob.ccw_socket);
      } else {

      // Grab the lock to avoid sync issues
      pthread_mutex_lock(&lock);

      // All data transfers are preceded by a CCW.
      ccw.code  =  0x00;
      ccw.code  =  iob.buffer[0];
      ccw.flags =  iob.buffer[4];
      ccw.chain =  iob.buffer[5];
      ccw.count = (iob.buffer[6] << 8) | iob.buffer[7];
      Eregs_Inp[0x61] = ccw.code;           // Set Chan command in INSEADCM

      printf("\nCA1: Channel Command: %02X, length: %d, Flags: %02X, Chained: %02X \n\r", ccw.code, ccw.count,ccw.flags,ccw.chain);
      /* Send an ACK to the host */
      send_ack(iob.ccw_socket);

      // Check and process channel command.
      switch (ccw.code) {
         case 0x00:       // Test I/O ?
            // Send channel end and device end to the host. Sufficient for now (might need to send x00).
            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            break;

         case 0x02:       // Read ?
            if ((Eregs_Inp[0x67] & 0x0008) == 0x00) {        // If channel not enabled
               while (reg_bit(0x67, 0x0008) == OFF) wait();  // Wait until channel enable is allowed
               Eregs_Inp[0x67] |= 0x000C;                    // Enable channel and subchannel
            }

            while (Ireg_bit(0x77, 0x0018) == ON) wait();     // Wait for initial/data selection reset
            Eregs_Inp[0x60] |= 0x8000;                       // Set initial selection
            Eregs_Inp[0x62] |= 0x8000;                       // Set outbound data transfer request
            Eregs_Inp[0x77] |= 0x0008;                       // Set Initial select lvl 3 interrupt
            CA1_IS_req_L3 = ON;                              // Chan Adap Initial Sel request flag

            while (Ireg_bit(0x77, 0x008) == ON) wait();      // Wait for initial selection reset
            i = 0;                                           // Data to be send counter

            // Need to insert test for fault status condition
            while ((Eregs_Out[0x62] & 0x8000) == 0x8000) {         // While data transfer request
               nobytes = (Eregs_Out[0x62] & 0x0003);
               switch (nobytes) {
                  case 0x01:
                     data_buffer[i] = Eregs_Out[0x64] >> 8;        // Load outbound data bytes 1
                     i = i + 1;
                     break;
                  case 0x02:
                     data_buffer[i] = Eregs_Out[0x64] >> 8;        // Load outbound data bytes 1
                     data_buffer[i+1] = Eregs_Out[0x64] & 0x00FF;  // Load outbound data bytes 2
                     i = i + 2;
                     break;
                  case 0x03:
                     data_buffer[i] = Eregs_Out[0x64] >> 8;        // Load outbound data bytes 1
                     data_buffer[i+1] = Eregs_Out[0x64] & 0x00FF;  // Load outbound data bytes 2
                     data_buffer[i+2] = Eregs_Out[0x65] >> 8;      // Load outbound data bytes 3
                     i = i + 3;
                     break;
                  case 0x00:
                     data_buffer[i] = Eregs_Out[0x64] >> 8;        // Load outbound data bytes 1
                     data_buffer[i+1] = Eregs_Out[0x64] & 0x00FF;  // Load outbound data bytes 2
                     data_buffer[i+2] = Eregs_Out[0x65] >> 8;      // Load outbound data bytes 3
                     data_buffer[i+3] = Eregs_Out[0x65] & 0x00FF;  // Load outbound data bytes 2
                     i = i + 4;
                     break;
               }
               Eregs_Inp[0x77] |= 0x0010;                          // Set L3 Data Service Request
               CA1_DS_req_L3 = ON;                                 // Chan Adap Data Service request flag
               while (Ireg_bit(0x77, 0x0010) == ON) wait();        // Wait for reset of Data/Status interrupt
            }

//          print_hex(data_buffer, i);

            rc = send(iob.ccw_socket,(void*)&data_buffer, i,0);
            // Wait for the ACK from the host
            recv_ack(iob.ccw_socket);

            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            Eregs_Inp[0x62] &= ~0x8000;                      // reset outbound data transfer
            break;

         case 0x03:       // NO-OP ?
            // Send channel end and device end to host. Sufficient for now (might need to send x00).
            carnstat = 0x00;
            carnstat = (carnstat | CSW_CEND) | CSW_DEND;
            rc = send(iob.ccw_socket,(void*)&carnstat, 1,0);
            // Wait for the ACK from the host
            recv_ack(iob.ccw_socket);
            printf("CARNSTAT: %04X via socket %d\n\r", carnstat, iob.ccw_socket);
            break;

         case 0x04:       // Sense ?
            if ((Eregs_Inp[0x67] & 0x0008) == 0x00)    {     // If channel not enabled
               while (reg_bit(0x67, 0x0008) == OFF) wait();  // Wait until channel enable is allowed
               Eregs_Inp[0x67] |= 0x000C;                    // Enable channel and subchannel
            }
            while (Ireg_bit(0x77, 0x0018) == ON) wait();     // Wait for selection reset
            Eregs_Inp[0x60] |= 0x8000;                       // Set initial selection
            Eregs_Inp[0x77] |= 0x0008;                       // Set Initial select lvl 3 interrupt
            CA1_IS_req_L3 = ON;                              // Chan Adap Initial Sel request flag
            while (Ireg_bit(0x77, 0x0008) == ON) wait();     // Wait for initial selection reset

            nobytes = (Eregs_Out[0x62] & 0x0003);            // Get nr of bytes
            switch (nobytes) {
               case 0x01:
                  data_buffer[0] = Eregs_Out[0x64] >> 8;     // Load sense data byte 0
                  printf("CA1: Sending sense Byte 0 %02X \n\r", data_buffer[0]);
                  break;
               case 0x02:
                  data_buffer[0] = Eregs_Out[0x64] >> 8;     // Load sense data byte 0
                  data_buffer[1] = Eregs_Out[0x64] & 0x00FF; // Load sense data byte 1
                  printf("CA1: Sending sense Byte 0 %02X, Byte 1 %02X \n\r",
                         data_buffer[0], data_buffer[1]);
                  break;
            }

            rc = send_socket(iob.ccw_socket,(void*)&data_buffer, nobytes);
            // Wait for the ACK from the host
            recv_ack(iob.ccw_socket);

            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            break;

         case 0x05:       // IPL command
         case 0x01:       // Write
         case 0x09:       // Write Break
            //  Should return x'00'

            if ((Eregs_Inp[0x67] & 0x0008) == 0x00) {        // If channel not enabled
               while (reg_bit(0x67, 0x0008) == OFF) wait();  // Wait until channel enable is allowed
               Eregs_Inp[0x67] |= 0x000C;                    // Enable channel and subchannel
            }

            while (Ireg_bit(0x77, 0x0018) == ON) wait();     // Wait for selection reset
            Eregs_Inp[0x60] |= 0x8000;                       // Set initial selection
            Eregs_Inp[0x77] |= 0x0008;                       // Set Initial select lvl 3 interrupt
            CA1_IS_req_L3 = ON;                              /* Chan Adap Initial Sel request flag */
            while (Ireg_bit(0x77, 0x0008) == ON) wait();     // Wait for initial selection reset

            Eregs_Inp[0x62] &= ~0x0400;                      // Reset channel stop
            Eregs_Inp[0x62] |= 0x4000;                       // Set inbound data transfer request

            // Read data from host

            rc = recv( iob.ccw_socket, iob.buffer, sizeof(iob.buffer),0);
            printf("CA1: received: %d  bytes from host\n\r",rc);
            // Send an ACK to the host
            send_ack(iob.ccw_socket);
            iob.bufferl = rc;

            // ************************************************************
            // Data transfer loop starts here
            // ************************************************************
            print_hex(iob.buffer, iob.bufferl);
            i = 0;
            while (i < iob.bufferl) {
               nobytes = (Eregs_Out[0x62] & 0x0003);
               while (Ireg_bit(0x77, 0x0010) == ON) wait();  // MIROS07 (see miniROS listing)
                  tcount = nobytes;                          // Set number of bytes to transfer
                  if (tcount == 0)
                     tcount = 0x0004;                        // 0x00 means 4 bytes to transfer
                  if ((iob.bufferl - i) < tcount) {          // Remaining bytes < requested bytes
                     nobytes = iob.bufferl - i;
                     tcount = nobytes;
                  }
                  switch (nobytes) {
                     case 0x01:
                        Eregs_Inp[0x64] = iob.buffer[i];     // Load inbound data bytes 1
                        i = i + 1;
                     case 0x02:
                        Eregs_Inp[0x64] = iob.buffer[i];     // Load inbound data bytes 1 and 2
                        Eregs_Inp[0x64] = ((Eregs_Inp[0x64] << 8) | iob.buffer[i+1]) & 0xFFFF;
                        i = i + 2;
                        break;
                     case 0x03:
                        Eregs_Inp[0x64] = iob.buffer[i];     // Load inbound data bytes 1 and 2
                        Eregs_Inp[0x64] = ((Eregs_Inp[0x64] << 8) | iob.buffer[i+1]) & 0xFFFF;
                        Eregs_Inp[0x65] = iob.buffer[i+2];   // Load inbound data bytes 3 and 4
                        i = i + 3;
                        break;
                     case 0x00:
                        Eregs_Inp[0x64] = iob.buffer[i];     // Load inbound data bytes 1 and 2
                        Eregs_Inp[0x64] = ((Eregs_Inp[0x64] << 8) | iob.buffer[i+1]) & 0xFFFF;
                        Eregs_Inp[0x65] = iob.buffer[i+2];   // Load inbound data bytes 3 and 4
                        Eregs_Inp[0x65] = ((Eregs_Inp[0x65] << 8) | iob.buffer[i+3]) & 0xFFFF;
                        i = i + 4;
                        tcount = 0x0004;
                        break;
                   }
                   Eregs_Out[0x62] &= ~0x0600;               // Reset Reg 62 bits
                   Eregs_Inp[0X62] = (Eregs_Inp[0X62] & ~0x0007) | tcount;  // Set number of bytes transferred
                   Eregs_Inp[0x77] |= 0x0010;                // Set L3 Data Service Request
                   CA1_DS_req_L3 = ON;                       // Chan Adap Data Service request flag */
 //                printf("\nCA1: Transfer %04X, Data = %04X \n\r",  i, Eregs_Inp[0x64]);
               }

            printf("CA1: Data transfer complete...\n\r");
            while (Ireg_bit(0x77, 0x0010) == ON) wait();     // MIROS07 (See miniROS listing)
            Eregs_Inp[0x62] |= 0x0400;                       // Set channel stop
            Eregs_Inp[0X62] &= ~0x0007;                      // Set number of bytes transferred to 0
            Eregs_Inp[0x77] |= 0x0010;                       // Set data/serv lvl 3 interrupt
            CA1_DS_req_L3 = ON;                              /* Chan Adap Data Service request flag */
            printf("CA1: Channel Stop\n\r");

            if (Eregs_Out[0x62] & 0x1000) {                  // Present Channel end
               printf("CA1: Present NSC channel end status \n\r");
               carnstat = (carnstat | CSW_CEND) | CSW_DEND;
            }
            else {
               while (Ireg_bit(0x77, 0x0010) == ON) wait();  // Wait for reset Data/Serv l3
               Eregs_Inp[0x62] &= ~0x4000;                   // Reset inbound data transfer
               carnstat = (Eregs_Out[0x66] & 0x00FF);
            }
            Eregs_Inp[0x62] &= ~0x04D0;                      // Reset chan stop, Sel Reset, Bus out check, Stacked status

            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            break;

         case 0x31:         // Initial Write
         case 0x51:         // Write start 1
         case 0x32:         // Initial Read
         case 0x52:         // Read start 1
         case 0x93:         // Reset command
            if ((Eregs_Inp[0x67] & 0x0008) == 0x00) {        // If channel not enabled
               while (reg_bit(0x67, 0x0008) == OFF) wait();  // Wait until channel enable is allowed
               Eregs_Inp[0x67] |= 0x000C;                    // Enable channel and subchannel
            }

            while (Ireg_bit(0x77, 0x0018) == ON) wait();     // Wait for selection reset
            Eregs_Inp[0x60] |= 0x8000;                       // Set initial selection
            Eregs_Inp[0x77] |= 0x0008;                       // Set Initial select lvl 3 interrupt
            CA1_IS_req_L3 = ON;                              // Chan Adap Initial Sel request flag
            while (Ireg_bit(0x77, 0x0008) == ON) wait();     // Wait for initial selection reset

            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            break;

         default:
            // Send CA return status to host
            send_carnstat(iob.ccw_socket,&carnstat,&ackbuf);
            break;
      }    /* End of switch (ccw.code) */

         /* Release the lock */
         pthread_mutex_unlock(&lock);
      } /* End of if - else */
   }    /* End of while(1)... */

// close socket removed
   return 0;
}


// ************************************************************
// This subroutine test for 1 bit in a External Output reg.
// If '0' OFF is returned, if 1 'ON' returned.
// ************************************************************
int reg_bit(int reg, int bit_mask) {
   if ((Eregs_Out[reg] & bit_mask) == 0x00)
      return(OFF);
   else
      return(ON);
}

// ************************************************************
// This subroutine test for 1 bit in a External Input reg.
// If '0' OFF is returned, if 1 'ON' returned.
// ************************************************************
int Ireg_bit(int reg, int bit_mask) {
   if ((Eregs_Inp[reg] & bit_mask) == 0x00)
      return(OFF);
   else
      return(ON);
}

// ************************************************************
// This subroutine waits 1 usec
// ************************************************************
void wait() {
   usleep(1);
   return;
}

