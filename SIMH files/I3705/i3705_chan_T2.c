/* 3705_chan_T2.c IBM 3705 Channel Adaptor Type 2 simulator

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
*/

#include "sim_defs.h"
#include "i3705_defs.h"
#include "i3705_Eregs.h"     // Exernal regs defs
#include <signal.h>
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#define IMAX 4096            // Input buffer size Random. Need to define a more rational value
#define RMAX 64              // Response buffer size Random. Need to define a more rational value

#define CAA 0                // Channel Adapter channel connection A
#define CAB 1                // Channel Adapter channel connection B
#define MAXPORTS 4
#define PORTCA1A 37051       // TCP/IP port number CA1 A
#define PORTCA1B 37052       // TCP/IP port number CA1 B
#define PORTCA2A 37053       // TCP/IP port number CA2 A
#define PORTCA2B 37054       // TCP/IP port number CA2 B
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

#define checkrc(expr) if(!(expr)) { perror(#expr); return -1; }

typedef enum { false, true } bool;

extern int32 Eregs_Inp[];
extern int32 Eregs_Out[];
extern uint8 M[];
extern int8  CA1_DS_req_L3;  /* Chan Adap Data/Status request flag */
extern int8  CA1_IS_req_L3;  /* Chan Adap Initial/Sel request flag */

void *CAx_thread(void *args);
void *CA_ATTN_thread(void *args);

char data_buffer[IMAX];
char response_buffer[RMAX];
int i;
uint8_t nobytes, tcount;

// Declaration of thread condition variable
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

// Declaring mutex
pthread_mutex_t lock;

int reg_bit(int reg, int bit_mask);
int Ireg_bit(int reg, int bit_mask);
void wait();

struct CCW   /* Channel Command Word */
   {
   uint8_t  code;
   uint8_t  dataddress[3];
   uint8_t  flags;
   uint8_t  chain;           // Format 0. Chain is not correct (yet), but aligned to Hercules
   uint16_t count;
} ccw;

struct CSW   /* Channel Status Word */
   {
   uint8_t  key;
   uint8_t  dataddress[3];
   uint8_t  unit_conditions;
   uint8_t  channel_conditions;
   uint16_t count;
} csw;

struct IO3705 {
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

struct pth_args
   {
   void* arg1;
   void* arg2;
} *args;

int wrkid, offset;

// ************************************************************
// Function to format and display incomming data from host
// ************************************************************
void print_hex(char *buffptr,int buf_len) {
   printf("\nRecord length: %X  (hex)\n\r", buf_len);
      for (int i = 0; i < buf_len; i++) {
         printf("%02X ", (unsigned char)buffptr[i]);
         if ((i + 1) % 16 == 0)
            printf("\n\r");
      }
      printf("\n\r");
   return;
}

// ************************************************************
// Function to check if socket is (still) connected
// ************************************************************
static bool IsSocketConnected(int sockfd) {
   int rc;
   struct sockaddr_in *ccuptr;
   socklen_t *addrlen;
   ccuptr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
   addrlen = (socklen_t*)malloc(sizeof(socklen_t));

   rc = getpeername(sockfd, ccuptr, addrlen);

   free(ccuptr);
   free(addrlen);

   if (rc == 0)
      return true;
   else
      return false;
}


// ************************************************************
// Function to accept TCP connection from host
// ************************************************************
int host_connect(struct IO3705 *iob, int abport) {
   int alive = 1;     // Enable KEEP_ALIVE
   int idle = 5;      // First  probe after 5 seconds
   int intvl = 3;     // Subsequent probes after 3 seconds
   int cntpkt = 3;    // Timeout after 3 failed probes
   int timeout = 1000;
   int rc;

   // Accept the incoming connection
   iob->addrlen[abport] = sizeof(iob->address[abport]);

   iob->bus_socket[abport] = accept(iob->CA_socket[abport], (struct sockaddr *)&iob->address[abport], (socklen_t*)&iob->addrlen[abport]);
   if (iob->bus_socket[abport] < 0) {
      printf("\nCA%c: Host accept failed for bus connection...\r",iob->CA_id);
      return -1;
   } else {

      if (setsockopt(iob->bus_socket[abport], SOL_SOCKET, SO_KEEPALIVE, (void *)&alive, sizeof(alive))) {
         perror("ERROR: setsockopt(), SO_KEEPALIVE");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPIDLE, (void *)&idle, sizeof(idle))) {
         perror("ERROR: setsockopt(), SO_KEEPIDLE");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPINTVL, (void *)&intvl, sizeof(intvl))) {
         perror("ERROR: setsockopt(), SO_KEEPINTVL");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPCNT, (void *)&cntpkt, sizeof(cntpkt))) {
         perror("ERROR: setsockopt(), SO_KEEPCNT");
         return -1;
      }

      printf("\nCA%c: New bus connection on 3705 port %d, socket fd is %d, ip is : %s, port : %d \n\r",
            iob->CA_id,iob->CA_socket[abport], iob->bus_socket[abport], inet_ntoa(iob->address[abport].sin_addr),
            (ntohs(iob->address[abport].sin_port)));
   }
   ;fcntl(iob->bus_socket[abport],F_SETFL, fcntl(iob->bus_socket[abport],F_GETFL) & ~O_NONBLOCK);


   // Get the tag connection
   while (1) {
      iob->tag_socket[abport] = accept(iob->CA_socket[abport], (struct sockaddr *)&iob->address[abport], (socklen_t*)&iob->addrlen[abport]);

      if (iob->tag_socket[abport] > 0) {
         printf("\nCA%c: New tag connection on 3705 port %d, socket fd is %d, ip is : %s, port : %d \n\r",
                  iob->CA_id,iob->CA_socket[abport], iob->tag_socket[abport], inet_ntoa(iob->address[abport].sin_addr),
                  (ntohs(iob->address[abport].sin_port)));
         break;
      } else {
         if (errno != EAGAIN)  {
            printf("\nCA%c: Host accept failed with errno %d for tag connection...\n\r",iob->CA_id,errno);
            return -1;
         }
      }
   }
   return 0;
}


// ************************************************************
// Function to send CA return status to the host
// ************************************************************
void send_carnstat(int sockptr, char *carnstat, char *ackbuf, char CA_id) {
   int rc;                      /* Return code */

   while (Ireg_bit(0x77, 0x0028) == ON)
       wait();     // Wait for CA 1 L3 interrupt reset

   // If DE and CE and reset Write Break Remember and Channel Active
   if (*carnstat & CSW_DEND) {
      Eregs_Inp[0x55] &= ~0x0040;  // Reset Write Break Remember
      Eregs_Inp[0x55] &= ~0x0100;  // Reset Channel Active
      *carnstat |= CSW_CEND;       // CA sets channel end
   }
   printf("CA%c: CARNSTAT %04X via socket %d\n\r", CA_id, *carnstat, sockptr);
   if (sockptr != -1) {
      rc = send(sockptr, carnstat, 1, 0);
      printf("CA%c: Send %d bytes on socket %d\n\r", CA_id, rc, sockptr);
   } else
      rc = -1;

   if (rc < 0) {
      printf("\nCA%c: CA status send to host failed...\n\r",CA_id);
      return;
   }
   // Wait for the ACK from the host
   rc = recv(sockptr, ackbuf, 1,0);
   printf("CA%c: Ack received %02X on socket %d\n\r",CA_id,*ackbuf,sockptr);
   Eregs_Out[0x54] &= ~0xFFFF;                      // Reset CA status bytes
   if (CA_id=='1')
      Eregs_Inp[0x55] &= ~0x0101;                   // Reset CA Active and CA 1 selected
   else
      Eregs_Inp[0x55] &= ~0x0102;                   // Reset CA Active  and CA 2 selected
   return;
}  /* end function send_carnstat */


// ************************************************************
// Function to wait for an ACK from the host
// ************************************************************
void recv_ack(int sockptr) {
   char ackbuf;
   int rc;
   rc = read(sockptr, &ackbuf, 1);
   return;
}


// ************************************************************
// Function to send an ACK to the host
// ************************************************************
void send_ack(int sockptr) {
   char ackbuf;
   int rc;
   rc = send(sockptr, &ackbuf, 1, 0);
   return;
}


// ************************************************************
// Function to read data from TCP socket
// ************************************************************
int read_socket(int sockptr, char *buffptr, int buffsize) {
   int reclen;
   bzero(buffptr, buffsize);
   reclen = read(sockptr, buffptr, buffsize);
   return reclen;
}


// ************************************************************
// Function to close TCP socket
// ************************************************************
int close_socket(struct IO3705 *iob, int abchannel) {
   close(iob->bus_socket[abchannel]);
   close(iob->tag_socket[abchannel]);
   iob->bus_socket[abchannel] = -1;
   iob->tag_socket[abchannel] = -1;
   printf("\nCA%c: Channel connection A failed\n\r", iob->CA_id);
   return 0;
}


// ************************************************************
// Function to send response data to the host
// ************************************************************
int send_socket(int sockptr, char *respp, int respsize) {
   int rc;                      /* Return code */
   rc = send (sockptr, respp, respsize, 0);

   if (rc < 0) {
      printf("\nCA1: Send to host failed...\n\r");
      return -1;
   }
   return 0;
}  /* end function send_socket */


// ************************************************************
// Thread for sending Attention interrupts to the host
// or Level 3 interrupts to the NCP
// ************************************************************

void *CA_ATTN_thread(void *pthrargs) {
   struct IO3705 *iob, *iob1, *iob2;
   struct pth_args *args = pthrargs;
   iob1 = args->arg1;
   iob2 = args->arg2;
   int rc;
   char carnstat, ackbuf;
   ackbuf = 0x00;

   while (1) {
      while (iob1->CA_active == FALSE && iob2->CA_active == FALSE)
          wait();

      if (((Eregs_Out[0x55] & 0x0200) == 0x0200) || ackbuf == 0xF8 ) {
         // Grab the lock to avoid sync issues
         pthread_mutex_lock(&lock);
         // Determine which CA needs to react
         if ((Eregs_Out[0x57] & 0x0008) == 0x0008)
            iob = iob1;
         else
            iob = iob2;
         printf("CA%c: L3 register 55 %04X \n\r", iob->CA_id,Eregs_Out[0x55]);
         Eregs_Inp[0x55] |= 0x0200;               // Set Program Requested Attention
         Eregs_Inp[0x77] |= iob->CA_mask;         // Set CA1 L3 Interrupt Request
         CA1_IS_req_L3 = ON;                      // Chan Adap L3 Interrupt request flag
         while (Ireg_bit(0x77, iob->CA_mask) == ON)
            wait();
         Eregs_Out[0x55] &= ~0x0200;              // Reset attention request
         printf("CA%c: Sending Return status\n\r", iob->CA_id);
         // Send CA retun status to host
         send_carnstat(iob->tag_socket[iob->abswitch], &carnstat, &ackbuf,iob->CA_id);
         // Release the lock
         pthread_mutex_unlock(&lock);
      }
      // Check if the 3705 has requested the CA to request a L3 interrupt
      if ((Eregs_Out[0x57] & 0x0080) == 0x0080) {
         // Grab the lock to avoid sync issues
         pthread_mutex_lock(&lock);
         // Determine which CA needs to react
         if ((Eregs_Out[0x57] & 0x0008) == 0x0008)
           iob = iob1;
         else
           iob = iob2;
         printf("CA%c: L3 register 57 %04X \n\r", iob->CA_id, Eregs_Out[0x57]);
         while (Ireg_bit(0x77, iob->CA_mask) == ON)
            wait();
         Eregs_Inp[0x55] |= 0x0800;                       // Set Program Requested L3 interrupt
         Eregs_Out[0x55] |= 0x3000;                       // Set INCWAR and OUTCWAR valid for IPL
         Eregs_Inp[0x77] |= iob->CA_mask;                 // Set CA L3 interrupt request
         CA1_IS_req_L3 = ON;
         printf("CA%c: Requested L3 interrupt\n\r", iob->CA_id);
         while (Ireg_bit(0x77, iob->CA_mask) == ON) wait();
            wait();
         Eregs_Out[0x57] &= ~0x0080;              // Reset attention request

         // Send CA retun status to host
         // Release the lock
         pthread_mutex_unlock(&lock);
      }
      usleep(10);
   }
   return 0;
}


// ********************************************************************
// Channel adaptor 2 thread
// ********************************************************************
void *CA_T2_thread(void *arg) {
   int rc, sig, event_count;
   uint32_t CAx_tid[2];
   struct sockaddr_in address;
   typedef union epoll_data {
      void    *ptr;
      int      fd;
      uint32_t u32;
      uint64_t u64;
   } epoll_Data_t;

   struct epoll_event event, events[MAXPORTS];

   pthread_t id1,id2,id3;
   args = malloc(sizeof(struct pth_args) * 1);
   iob1 = malloc(sizeof(struct IO3705) * 1);
   iob2 = malloc(sizeof(struct IO3705) * 1);
   iob1->CA_id = '1';
   iob1->CA_mask = 0x0008;        // CA 1 select mask
   iob2->CA_id = '2';
   iob2->CA_mask = 0x0020;        // CA 2 select mask
   args->arg1 = iob1;
   args->arg2 = iob2;



   //  Indicate that this 3705 is equipped with a type 2 CA
   Eregs_Inp[0x79] &= ~0x0002;    // CA type 2 or 3 installed

   // Create and verify the A B sockets for CA1 and CA2
   while (reg_bit(0x79, 0x0010) == OFF)
      wait();  // Wait for ROS to reset test mode

   if ((iob1->CA_socket[CAA] = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
   }
   if ((iob1->CA_socket[CAB] = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
   }
   if ((iob2->CA_socket[CAA] = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
   }
   if ((iob2->CA_socket[CAB] = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
   }

   // Type of socket created CA 1
   iob1->address[CAA].sin_family = AF_INET;
   iob1->address[CAA].sin_addr.s_addr = INADDR_ANY;
   iob1->address[CAA].sin_port = htons( PORTCA1A );
   iob1->address[CAB].sin_family = AF_INET;
   iob1->address[CAB].sin_addr.s_addr = INADDR_ANY;
   iob1->address[CAB].sin_port = htons( PORTCA1B );

   // Type of socket created CA 2
   iob2->address[CAA].sin_family = AF_INET;
   iob2->address[CAA].sin_addr.s_addr = INADDR_ANY;
   iob2->address[CAA].sin_port = htons( PORTCA2A );
   iob2->address[CAB].sin_family = AF_INET;
   iob2->address[CAB].sin_addr.s_addr = INADDR_ANY;
   iob2->address[CAB].sin_port = htons( PORTCA2B );


   // Set all socket to allow multiple connections
   int flag = 1;
   if (-1 == setsockopt(iob1->CA_socket[CAA], SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA1: Setsockopt failed for Channel A\n\r");
   }
   if (-1 == setsockopt(iob1->CA_socket[CAB], SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA1: Setsockopt failed for Channel B\n\r");
   }
   if (-1 == setsockopt(iob2->CA_socket[CAA], SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA2: Setsockopt failed for Channel A\n\r");
   }
   if (-1 == setsockopt(iob2->CA_socket[CAB], SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA2: Setsockopt failed for Channel B\n\r");
   }

   // Bind the socket to localhost port PORT
   if (bind(iob1->CA_socket[CAA], (struct sockaddr *)&iob1->address[CAA], sizeof(iob1->address[CAA])) < 0) {
      perror("\nCA1: bind failed for channel A\n\r");
      exit(EXIT_FAILURE);
   }
   if (bind(iob1->CA_socket[CAB], (struct sockaddr *)&iob1->address[CAB], sizeof(iob1->address[CAB])) < 0) {
      perror("\nCA1: bind failed for channel B\n\r");
      exit(EXIT_FAILURE);
   }
   if (bind(iob2->CA_socket[CAA], (struct sockaddr *)&iob2->address[CAA], sizeof(iob2->address[CAA])) < 0) {
      perror("\nCA2: bind failed for channel A\n\r");
      exit(EXIT_FAILURE);
   }
   if (bind(iob2->CA_socket[CAB], (struct sockaddr *)&iob2->address[CAB], sizeof(iob2->address[CAB])) < 0) {
      perror("\nCA2: bind failed for channel B\n\r");
      exit(EXIT_FAILURE);
   }
   // Now server is ready to listen
   printf("CA_T2: Waiting for channel connections on TCP ports %d, %d, %d, %d \n\r", PORTCA1A, PORTCA1B, PORTCA2A, PORTCA2B);

   // Listen and verify
   if ((listen(iob1->CA_socket[CAA], 5)) != 0) {
      printf("\nCA1: Listen failed...\n\r");
      exit(0);
   }
   if ((listen(iob1->CA_socket[CAB], 5)) != 0) {
      printf("\nCA1: Listen failed...\n\r");
      exit(0);
   }
   if ((listen(iob2->CA_socket[CAA], 5)) != 0) {
      printf("\nCA2: Listen failed...\n\r");
      exit(0);
   }
   if ((listen(iob2->CA_socket[CAB], 5)) != 0) {
      printf("\nCA2: Listen failed...\n\r");
      exit(0);
   }


int epoll_fd = epoll_create(10);
   if (epoll_fd == -1) {
      printf("\nCA_T2: failed to created epoll file descriptor\n\r");
      return 0;
   }

   iob1->CA_active = FALSE;
   iob2->CA_active = FALSE;
   pthread_create(&id3, NULL, CA_ATTN_thread, args);

   printf("\nCA_T2: Activating polling for channel connection requests...\n\r");

   // Add polling events for all 4 ports
   event.events = EPOLLIN | EPOLLONESHOT;
   event.data.fd = iob1->CA_socket[CAA];
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, iob1->CA_socket[CAA], &event)) {
      printf("\nAdd polling event failed for CA1-A %d\n\r",errno);
      close(epoll_fd);
      return 0;
   }
   event.events = EPOLLIN | EPOLLONESHOT;
   event.data.fd = iob1->CA_socket[CAB];
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, iob1->CA_socket[CAB], &event)) {
      printf("\nAdd polling event failed for CA1-B %d\n\r",errno);
      close(epoll_fd);
      return 0;
   }
   event.events = EPOLLIN | EPOLLONESHOT;
   event.data.fd = iob2->CA_socket[CAA];
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, iob2->CA_socket[CAA], &event)) {
      printf("\nAdd polling event failed for CA2-A %d\n\r",errno);
      close(epoll_fd);
      return 0;
   }
   event.events = EPOLLIN | EPOLLONESHOT;
   event.data.fd = iob2->CA_socket[CAB];
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, iob2->CA_socket[CAB], &event)) {
      printf("\nAdd polling event failed for CA2-B %d\n\r",errno);
      close(epoll_fd);
      return 0;
   }

   while(1) {
      if (!IsSocketConnected(iob1->CA_socket[CAA])) {
         event.events = EPOLLIN | EPOLLONESHOT;
         event.data.fd = iob1->CA_socket[CAA];
         if (epoll_ctl(epoll_fd, EPOLL_CTL_MOD, iob1->CA_socket[CAA], &event)) {
            printf("\nModifying polling event failed for CA1-A %d\n\r", errno);
            close(epoll_fd);
            return 0;
         }
      }

      if (!IsSocketConnected(iob1->CA_socket[CAB])) {
         event.events = EPOLLIN | EPOLLONESHOT;
         event.data.fd = iob1->CA_socket[CAB];
         if (epoll_ctl(epoll_fd, EPOLL_CTL_MOD, iob1->CA_socket[CAB], &event)) {
            printf("\nModifying polling event failed for CA1-B %d\n\r", errno);
            close(epoll_fd);
            return 0;
         }
      }
      if (!IsSocketConnected(iob2->CA_socket[CAA])){
         event.events = EPOLLIN | EPOLLONESHOT;
         event.data.fd = iob2->CA_socket[CAA];
         if (epoll_ctl(epoll_fd, EPOLL_CTL_MOD, iob2->CA_socket[CAA], &event)) {
            printf("\nModifying polling event failed for CA2-A %d\n\r", errno);
            close(epoll_fd);
            return 0;
         }
      }
      if (!IsSocketConnected(iob2->CA_socket[CAB])){
         event.events = EPOLLIN | EPOLLONESHOT;
         event.data.fd = iob2->CA_socket[CAB];
         if (epoll_ctl(epoll_fd, EPOLL_CTL_MOD, iob2->CA_socket[CAB], &event)) {
            printf("\nModifying polling event failed for CA2-B %d\n\r", errno);
            close(epoll_fd);
            return 0;
         }
      }

      event_count = epoll_wait(epoll_fd, events, MAXPORTS, 5000);

      for (int i = 0; i < event_count; i++) {
         printf("\nEvents received for %d\n\r",events[i].data.fd );
         if (events[i].data.fd == iob1->CA_socket[CAA]) {
            // Accept the incoming connection
            rc = host_connect(iob1, CAA);
            if (rc == 0) {
               iob1->abswitch = 0;
               // Get device number
               rc = read_socket(iob1->bus_socket[iob1->abswitch], iob1->buffer, sizeof(iob1->buffer));
               if (rc != 0) {
                  iob1->devnum = (iob1->buffer[0] << 8) | iob1->buffer[1];
                  printf("CA1: Connected to device %04X\n\r", iob1->devnum);

                  if (id1 != 0) {
                     sig = pthread_kill(id1, SIGCONT);
                     printf("CA1: Thread check result %d\n\r", sig);
                  }
                  if (id1 == 0 || sig != 0) {
                     args->arg1 = iob1;
                     pthread_create(&id1, NULL, CAx_thread, args);
                     printf("CA1: Started. Thread id %p\n\r", id1);
                     CAx_tid[0] = id1;
                  }

               } else { close_socket(iob1,CAA);
               }
            } else { close_socket(iob1,CAA);
         }
      } // End for stmt

      if (events[i].data.fd == iob1->CA_socket[CAB]) {
         // Accept the incoming connection
         rc = host_connect(iob1, CAB);
         if (rc == 0) {
            iob1->abswitch = 1;
            // Get device number
            rc = read_socket(iob1->bus_socket[iob1->abswitch], iob1->buffer, sizeof(iob1->buffer));
            if (rc != 0) {
               iob1->devnum = (iob1->buffer[0] << 8) | iob1->buffer[1];
               printf("CA%c: Connected to device %04X\n\r", iob1->CA_id, iob1->devnum);

               if (id1 != 0) {
                  sig  = pthread_kill(id1,SIGCONT);
                  printf("CA%c: Thread check result %d\n\r", iob1->CA_id, sig);
               }
               if (id1 == 0 || sig != 0) {
                  args->arg1 = iob1;
                  pthread_create(&id1, NULL, CAx_thread, args);
                  printf("CA%c: Started. Thread id %p\n\r", iob1->CA_id, id1);
                  CAx_tid[0] = id1;
               }
            } else {  close_socket(iob1,CAB);
            }
         } else { close_socket(iob1,CAB);
         }
      } // End if events[i].data.fd iob1 CAB

      if (events[i].data.fd == iob2->CA_socket[CAA]) {
         // Accept the incoming connection
         rc = host_connect(iob2, CAA);
         if (rc == 0) {
            iob2->abswitch = 0;
            // Get device number
            rc = read_socket(iob2->bus_socket[iob2->abswitch], iob2->buffer, sizeof(iob2->buffer));
            if (rc != 0) {
               iob2->devnum = (iob2->buffer[0] << 8) | iob2->buffer[1];
               printf("CA%c: Connected to device %04X\n\r", iob2->CA_id, iob2->devnum);

               if (id2 != 0) {
                  sig  = pthread_kill(id2,SIGCONT);
                  printf("CA%c: Thread check result %d\n\r", iob2->CA_id, sig);
               }
               if (id1 == 0 || sig != 0) {
                  args->arg1 = iob2;
                  pthread_create(&id2, NULL, CAx_thread, args);
                  printf("CA%c: Started. Thread id %p\n\r", iob2->CA_id, id2);
                  CAx_tid[1] = id2;
               }
            } else {  close_socket(iob2,CAA);
            }
         } else { close_socket(iob2,CAA);
         }
      }   // End if events[i].data.fd iob2 CAA

      }   // End for event_count
   }   // End While(1)

   if (close(epoll_fd)) {
      printf("\nCA_T2: failed to close epoll file descriptor\n\r");
      return 0;
   }
   return 0 ;
}


// ************************************************************
// The channel adaptor 1 / 2  handling thread starts here...
// ************************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */
void *CAx_thread(void *pthargs) {
   struct pth_args *args = pthargs;
   struct IO3705 *iob = args->arg1;

   int rc;
   int cc = 0;
   int sockfc = -1;
   int bufbase, condition;
   pthread_t id;
   char carnstat, ackbuf;
   uint16_t incwar,outcwar, wdcnt, wdcnttmp, wdcnttot, cacw1, cacw2;

   printf("CA%c: thread started sucessfully. ...Process ID %d \n\r", iob->CA_id, getppid());

   // Init the lock
   if (pthread_mutex_init(&lock, NULL) !=0)  {
      printf("\nCA%c: Lock initialization failed \n\r", iob->CA_id);
      exit(EXIT_FAILURE);
   }

   Eregs_Inp[0x77] &= ~0x0028;           // Reset CA1 L3 interrupt
   CA1_DS_req_L3 = OFF;                  // Chan Adap Data/Status request flag
   CA1_IS_req_L3 = OFF;                  // Chan Adap Initial Sel request flag
   Eregs_Inp[0x55]  = 0x0000;            // Reset CA control register
   Eregs_Inp[0x58] |= 0x0008;            // Enable CA I/F A
   Eregs_Inp[0x55] |= 0x0010;            // Flag System Reset

   // Change the CA status to active
   iob->CA_active = TRUE;

   while(1) {
      // We do this for ever and ever...
      /***************************************************************/
      /*    Read channel command from host                           */
      /*                                                             */
      /*    This is the raw version: it assumes no pending operation */
      /*    Channel status tests need to be added                    */
      /*                                                             */
      /***************************************************************/
      if ( iob->bus_socket[iob->abswitch] < 1) {
         printf("\nCA%c: Aborting due to loss of active channel connection...\n\r", iob->CA_id);
         return 0;
      }

      rc = read_socket( iob->bus_socket[iob->abswitch], iob->buffer, sizeof(iob->buffer));

      if (rc == 0) {
         // Host disconnected, get details and print it
         printf("\nCA%c: Error reading CCW, closing channel connection\n\r", iob->CA_id);
         // Change the CA status to inactive
         iob->CA_active = FALSE;

         // Close the bus and tag socket and mark for reuse
         close(iob->bus_socket[iob->abswitch]);
         close(iob->tag_socket[iob->abswitch]);

         iob->bus_socket[iob->abswitch] = -1;
         iob->tag_socket[iob->abswitch] = -1;
      } else {
         // Grab the lock to avoid sync issues
         pthread_mutex_lock(&lock);

         // All data transfers are preceded by a CCW.
         ccw.code  =  0x00;
         ccw.code  =  iob->buffer[0];
         ccw.flags =  iob->buffer[4];
         ccw.chain =  iob->buffer[5];
         ccw.count = (iob->buffer[6] << 8) | iob->buffer[7];

         Eregs_Inp[0x5A] = ccw.code << 8;      // Set Chan command in CA Data Buffer
         Eregs_Inp[0x5C] &= ~0xFFFF;           // Clear command flags CA Command Register
         printf("\nCA%c: Channel Command: %02X, length: %d, Flags: %02X, Chained: %02X \n\r",
                iob->CA_id, ccw.code, ccw.count, ccw.flags, ccw.chain);
         // Send an ACK to the host
         send_ack(iob->bus_socket[iob->abswitch]);

         // **************************************************************
         // Check and process channel command.
         // **************************************************************
         switch (ccw.code) {
            case 0x00:       // Test I/O
               Eregs_Inp[0x5C] |= 0x8000;                  // Set CA Command Register
               // Send channel end and device end to the host. Sufficient for now (might need to send x00).
               // Send CA return status to host
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
               send_carnstat(iob->bus_socket[iob->abswitch],&carnstat,&ackbuf,iob->CA_id);
               break;

            case 0x02:       // Read
               Eregs_Inp[0x55] |= 0x0100;                  // Set Channel Active
               Eregs_Inp[0x5C] |= 0x2000;                  // Set CA Command Register
               Eregs_Inp[0x53] &= 0x0000;                  // Reset sense register

               while (reg_bit(0x55, 0x1000) == OFF)
                  wait();                                  // Wait for OUTCWAR to become valid
               bufbase = 0;                                // Set buffer base...
               wdcnttot = 0;                               // ... we will need this in case of chaining

               do {   // While condition remains 0
                  condition = 0;
                  outcwar = Eregs_Out[0x51];
                  cacw1 = (M[outcwar] << 8) | M[outcwar+1] & 0x00FF;      // Get first half of CA Control word
                  wdcnt = (cacw1 >> 2) & 0x03FF;           // Fetch Counter
                  Eregs_Inp[0x52] &= 0x0000;               // Clear Byte Count Register
                  Eregs_Inp[0x52] = wdcnt;
                  cacw2 = ((M[outcwar+2] << 8) | M[outcwar+3] & 0x00FF)
                            + (cacw1 << 14);               // Get data fetch address
                  printf("OUTCWAR %04X, CW %02X%02X %02X%02X\n\r",
                          outcwar, M[outcwar], M[outcwar+1], M[outcwar+2], M[outcwar+3]);
                  Eregs_Out[0x51] = Eregs_Out[0x51] + 4;

                  Eregs_Inp[0x5C] |= 0x0080;               // Set command register to OUT Control Word
                  Eregs_Inp[0x59]  = cacw2;                // Load cycle steal address with data load start address
                  printf("CW %04X\n\r",cacw1);
                  printf("Fetch starts at %04X, count = %04X\n\r", cacw2, wdcnt);

                  wdcnttmp = wdcnt;                        // Bytes to be transferred for this CW
                  wdcnttot = wdcnttot + wdcnt;             // Total byte count
                  for (i = 0; i < wdcnttmp; i++) {
                     data_buffer[bufbase+i] = M[cacw2+i];  // Load data directly into memory
                     Eregs_Inp[0x59] = Eregs_Inp[0x59] + 1;  // Increment cycle steal counter
                     wdcnt = wdcnt - 1;                    // Decrement byte counter
                     Eregs_Inp[0x52] = Eregs_Inp[0x52] - 1;
                  }  // End for stmt
                  bufbase = bufbase + i;                   // Point after last byte stored in buffer

                  if (cacw1 & 0x4000) {                    // If OUT STOP
                     if ((cacw1 & 0x1000) && !(cacw1 & 0x2000))  // Chaining On, Zero Override Off
                        condition = 2;
                     if (!(cacw1 & 0x1000))                // Chaining Off
                        condition = 1;
                     if ((cacw1 & 0x3000) == 0x3000) {     // Chaning On, Zero Override On
                        condition = 0;
                        while (Ireg_bit(0x77, iob->CA_mask) == ON)
                           wait();                         // Wait for CA1 L3 interrupt reset
                        Eregs_Inp[0x77] |= iob->CA_mask;   // Set CA1 L3 interrupt
                        CA1_IS_req_L3 = ON;                // Chan Adap Initial Sel request flag
                        while (Ireg_bit(0x77, 0x008) == ON)
                           wait();                         // Wait for initial selection reset
                     }
                  } else {
                     if ((cacw1 & 0x1000) && !(cacw1 & 0x2000))  // Chaining On, Zero Override Off
                        condition = 0;
                     if (!(cacw1 & 0x3000))                // Chaining Off, Zero Override Off,
                        condition = 1;
                     if (cacw1 & 0x2000)                   // Zero Override On
                        condition = 3;
                  }
                  printf("Condition = %d\n\r", condition);

              }  while (condition == 0);   // End of do stmt.

              if (condition != 2) {
                 while (Ireg_bit(0x77, iob->CA_mask) == ON)
                    wait();                                // Wait for CA1 L3 interrupt reset
                 Eregs_Inp[0x77] |= iob->CA_mask;          // Set CA1  L3 interrupt
                 CA1_IS_req_L3 = ON;                       // Chan Adap Initial Sel request flag
                 while (Ireg_bit(0x77, 0x008) == ON)
                    wait();                                // Wait for initial selection reset
              }

              rc = send(iob->bus_socket[iob->abswitch], (void*)&data_buffer, wdcnttot,0);
              // Wait for the ACK from the host
              recv_ack(iob->bus_socket[iob->abswitch]);

              // Send CA return status to host
              if (condition != 3) {
                 carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
                 if (condition == 2)
                    carnstat = CSW_DEND;
                 send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
              }
              break;

            case 0x03:       // NO-OP ?
               Eregs_Inp[0x5C] |= 0x0100;                  // Set CA Command Register
               // Send channel end and device end to host. Sufficient for now (might need to send x00).
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for CA1 L3 interrupt request reset
               carnstat = 0x00;
               carnstat |= CSW_DEND;
               send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
               break;

            case 0x04:       // Sense ?
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for CA1 L3 reset
               Eregs_Inp[0x77] |= iob->CA_mask;            // Set CA1 L3 interrupt request
               CA1_IS_req_L3 = ON;                         // Chan Adap L3 request flag
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for L3 interrupt request reset
               data_buffer[0] = Eregs_Out[0x53] >> 8;      // Load sense data byte 0
               printf("CA%c: Sending sense Byte 0 %02X \n\r", iob->CA_id, data_buffer[0]);

               rc = send_socket(iob->bus_socket[iob->abswitch], (void*)&data_buffer, 1);
               // Wait for the ACK from the host
               recv_ack(iob->bus_socket[iob->abswitch]);

               // Send CA return status to host
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
               send_carnstat(iob->bus_socket[iob->abswitch],&carnstat,&ackbuf,iob->CA_id);
               break;

            case 0x05:       // IPL command
            case 0x01:       // Write
            case 0x09:       // Write Break
               Eregs_Inp[0x53] &= 0x0000;                  // Reset sense register
               switch (ccw.code) {
                  case 0x01:
                     Eregs_Inp[0x5C] |= 0x4000;            // Set CA Command Register
                     Eregs_Inp[0x55] |= 0x0100;            // Set Channel Active
                     break;
                  case 0x05:
                     Eregs_Inp[0x5C] |= 0x0001;            // Set CA Command Register
                     while (Ireg_bit(0x77, iob->CA_mask) == ON)
                         wait();                           // Wait for CA1 L3 request reset
                     Eregs_Inp[0x77] |= iob->CA_mask;      // Set CA1 L3 interrupt request
                     CA1_IS_req_L3 = ON;
                     break;
                  case 0x09:
                     Eregs_Inp[0x55] |= 0x0100;            // Set Channel Active
                     Eregs_Inp[0x5C] |= 0x0200;            // Set CA Command Register
                     Eregs_Inp[0x55] |= 0x0040;            // Set Write Break Remember flag
                     break;
               }  // End of nested switch ccw.code

               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                   wait();                                 // Wait for CA1 L3 Request reset

               // Read data from host
               rc = recv( iob->bus_socket[iob->abswitch], iob->buffer, sizeof(iob->buffer),0);
               printf("CA%c: received: %d bytes from host\n\r",iob->CA_id,rc);
               // Send an ACK to the host
               send_ack(iob->bus_socket[iob->abswitch]);
               iob->bufferl = rc;
               bufbase = 0;                                // Set buffer base.
                                                           // We will need this in case of chaining

               // ************************************************************
               // Data transfer loop starts here
               // ************************************************************
               print_hex(iob->buffer, iob->bufferl);

               while (iob->bufferl) {
                  do {   // While condition remains 0
                     condition = 1;
                     printf("InpReg 55 %04X, OutReg 55 %04X\n\r",Eregs_Inp[0x55],Eregs_Out[0x55]);
                     while (reg_bit(0x55, 0x2000) == OFF)      // Wait for INCWAR to become valid
                        wait();

                     incwar = Eregs_Out[0x50];
                     cacw1 = (M[incwar] << 8) | M[incwar+1] & 0x00FF;  // Get first half of CA Control word
                     if ((cacw1 & 0x1000) == 0x0000) {          // If chain bit is off ....
                        Eregs_Inp[0x55] &= ~0x2000;        // ...reset INCWAR valid latch ...
                        Eregs_Out[0x55] &= ~0x2000;        // ...in both IN and OUT reg
                     }
                     wdcnt = (cacw1 >> 2) & 0x03FF;        // Load Counter
                     cacw2 = ((M[incwar+2] << 8) | M[incwar+3] & 0x00FF)
                               + (cacw1 << 14);            // Get data load address
                     printf("INCWAR %04X, CW %02X%02X %02X%02X\n\r",incwar, M[incwar], M[incwar+1], M[incwar+2], M[incwar+3]);
                     Eregs_Out[0x50] = Eregs_Out[0x50] + 4;

                     Eregs_Inp[0x5C] |= 0x0020;            // Set command register to IN Control Word
                     Eregs_Inp[0x59] = cacw2;              // Load cycle steal address with data load start address
                     printf("CW %04X\n\r",cacw1);
                     printf("Load starts at %04X, count=%04X\n\r", cacw2, wdcnt);

                     wdcnttmp = wdcnt < iob->bufferl?wdcnt:iob->bufferl;

                     for (i = 0; i < wdcnttmp; i++) {
                        M[cacw2+i] = iob->buffer[bufbase+i]; // Load data directly into memory
                        Eregs_Inp[0x59] = Eregs_Inp[0x59] + 1;  // Increment cycle steal counter
                        wdcnt = wdcnt - 1;                  // Decrement byte counter
                     }  // End For
                     iob->bufferl = iob->bufferl - wdcnttmp;
                     bufbase = bufbase + i;                 // Buffer base points to start of remaing data
                     if ((cacw1 & 0x8000) && wdcnt == 0) {  // If IN and count zero
                        if ((cacw1 & 0x2000) == 0x2000)  {    // Zero Override On
                           Eregs_Inp[0x55] |= 0x4000;         // Set Zero Override in reg 55
                           condition = 0;
                           Eregs_Inp[0x77] |= iob->CA_mask;   // Set CA1 L3 interrupt request
                           CA1_IS_req_L3 = ON;                // Chan Adap L3 request flag
                           while (Ireg_bit(0x77, iob->CA_mask) == ON)
                              wait();
                        } else {
                           Eregs_Inp[0x55] &= ~0x4000;                 // Reset zero ovcerride flag
                           } // End Zero override on
                        if ((cacw1 & 0x3000) == 0x0000)      // Chaining Off, Zero Override Off
                           condition = 1;
                        if ((cacw1 & 0x3000) == 0x1000)  // Chaining On, Zero override off,  count zero
                           condition = 0;

                     }  // End if cacw1

                  } while (condition == 0);  // End of do stmt


               }  // End of while iob->bufferl

               printf("CA%c: Data transfer complete, loaded %04X, remainder %04X\n\r",
                       iob->CA_id, wdcnttmp,wdcnt);
               // If byte count is zero, and there is no chaining
               // we will send a L3 interrupt to to CCU, otherwise...
               // ...we will countinue loading data. In case of chaining, we will fetch a new CW

               Eregs_Inp[0x52] &= 0x0000;                  // Clear Byte Count Register
               Eregs_Inp[0x52] = wdcnt;                    // Load Register with Byte count
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for L3 interrupt reset
               wdcnt = 0x0000;                             // clear count

               Eregs_Inp[0x55] |= 0x0020;                  // Set channel stop
               Eregs_Inp[0x55] &= ~0x4000;                 // Reset zero ovcerride flag
               printf("CA%c: Channel Stop\n\r",iob->CA_id);
               Eregs_Inp[0x77] |= iob->CA_mask;            // Set CA1 L3 interrupt request
               CA1_IS_req_L3 = ON;
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for CA1 L3 Request reset
               if (condition != 2) {                       // If Zero overide is on
                  carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);   // Get CA return status
                  // Send CA return status to host
                  send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
               }
               break;

            case 0x31:         // Initial Write
            case 0x51:         // Write start 1
            case 0x32:         // Initial Read
            case 0x52:         // Read start 1
            case 0x93:         // Reset command

               Eregs_Inp[0x55] |= 0x0100;                  // Set Channel Active
               Eregs_Inp[0x5C] |= 0x0008;                  // Set non-standard command in CA Command Register
               Eregs_Inp[0x53] &= 0x0000;                  // Reset sense register

               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for CA1 L3 interrupt request reset
               Eregs_Inp[0x77] |= iob->CA_mask;            // Set CA1 L3 interrupt request
               CA1_IS_req_L3 = ON;                         // Chan Adap L3 interrupt request flag
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                  // Wait for L3 iterrupt request reset

               // Send CA return status to host
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF); // Get CA return status
               send_carnstat(iob->bus_socket[iob->abswitch],&carnstat,&ackbuf,iob->CA_id);
               break;

            default:
               // Send CA return status to host
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF); // Get CA return status
               send_carnstat(iob->bus_socket[iob->abswitch],&carnstat,&ackbuf,iob->CA_id);
               break;
         }  // End of switch (ccw.code)

         // Release the lock
         pthread_mutex_unlock(&lock);
      }  // End of if - else
   }  // End of while(1)... */
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

