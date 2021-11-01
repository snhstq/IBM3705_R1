/* i3705_client.C   (c) Copyright Max H. Parke, 2007-2012              */
/*                  (c) Copyright Henk Stegeman & Edwin Freekenhost    */

/***********************************************************************/
/*                                                                     */
/* i3705_client.c - (C) Copyright 2021 by HJS & EF                     */
/*                                                                     */
/* Based on comm3705.c - (C) Copyright 2007 by MHP <ikj1234i&yahoo.com */
/*                                                                     */
/* Loosely based on commadpt.c by Ivan Warren                          */
/*                                                                     */
/* This module appears to the 3705 as a 3274 cluster controller        */
/*                                                                     */
/* A very minimalistic SNA stack is implemented.  All received SNA     */
/* requests are responded to with a positive response.  Also, there's  */
/* enough code to enable a single SNA session to logon in LU1 (TTY)    */
/* mode.                                                               */
/*                                                                     */
/* FID2 is the only SNA header type supported.                         */
/*                                                                     */
/* A large amount of required SNA functionality is not present in this */
/* release.  There are no "state machines", "layers", "services",      */
/* chaining*, pacing, brackets, etc, etc, etc.  There are              */
/* probably more bugs than working functions...    Enjoy ;-)           */
/*                                                                     */
/* A better implementation might be to separate the SNA functions out  */
/* into an independent process, with communications over a full-duplex */
/* TCP/IP socket... We might also get rid of all the magic constants...*/
/*                                                                     */
/* New in release 0.02 -                                               */
/* - VTAM switched (dial) support                                      */
/* - New remote NCP capability                                         */
/* - SNA 3270 (LU2) support (*with RU chaining)                        */
/* - fixes for some bugs in 0.01                                       */
/* New in release 0.03 -                                               */
/* - don't process TTY lines until CR received                         */
/* New in release 0.04 -                                               */
/* - make debug messages optional                                      */
/*                                                                     */
/***********************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <string.h>
#include <ifaddrs.h>
#include "i3705_defs.h"
#include "htypes.h"
#include "i3705_sdlc.h"
#include "i3705_client.h"
#include "codepage.c"
#include <sys/syscall.h>

#if !defined(min)
#define  min(a,b)   (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C

extern FILE *trace;
extern int debug_reg;
extern int8 Rsp_buf;                // Status Response buffer
extern int Plen;                   // Length of PIU response
extern int8 last_lu;                // Last addressed lu

COMMADPT *ca;
void make_seq (COMMADPT * ca, BYTE * bufptr);
int write_socket( int fd, const void *_ptr, int nbytes );
int send_packet (int csock, BYTE *buf, int len, char *caption);

// Supported FMD NS Headers
static unsigned char R010201[3] = {0x01, 0x02, 0x01};  // CONTACT
static unsigned char R010202[3] = {0x01, 0x02, 0x02};
static unsigned char R010203[3] = {0x01, 0x02, 0x03};
static unsigned char R010204[3] = {0x01, 0x02, 0x04};
static unsigned char R010205[3] = {0x01, 0x02, 0x05};
static unsigned char R01020A[3] = {0x01, 0x02, 0x0A};
static unsigned char R01020B[3] = {0x01, 0x02, 0x0B};
static unsigned char R01020F[3] = {0x01, 0x02, 0x0F};
static unsigned char R010211[3] = {0x01, 0x02, 0x11};
static unsigned char R010216[3] = {0x01, 0x02, 0x16};
static unsigned char R010217[3] = {0x01, 0x02, 0x17};
static unsigned char R010219[3] = {0x01, 0x02, 0x19};  // ANA
static unsigned char R01021A[3] = {0x01, 0x02, 0x1A};
static unsigned char R01021B[3] = {0x01, 0x02, 0x1B};
static unsigned char R010280[3] = {0x01, 0x02, 0x80};
static unsigned char R010281[3] = {0x01, 0x02, 0x81};
static unsigned char R010284[3] = {0x01, 0x02, 0x84};  // REQCONT


/*-------------------------------------------------------------------*/
/* Positive RU's                                                     */
/*-------------------------------------------------------------------*/
char F2_ACTPU_Rsp[] = {
      0x11, 0x11, 0x40, 0x40,  0x40, 0x40, 0x40, 0x40,
      0x40, 0x40, 0x00, 0x00,  0x07, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00 };
char F2_ACTLU_Rsp[] = {
      0x0D, 0x01, 0x01, 0x00,  0x85, 0x00, 0x00, 0x00,
      0x0C, 0x0E, 0x03, 0x00,  0x01, 0x00, 0x00, 0x00,
      0x40, 0x40, 0x40, 0x40,  0x40, 0x40, 0x40, 0x40 };
char F2_DACTLU_Rsp[] = {
      0x0E };
char F2_BIND_Rsp[] = {
      0x31 };
char F2_SDT_Rsp[]  = {
      0xA0 };
char F2_CLEAR_Rsp[] = {
      0xA1 };
char F2_INITSELF_Req[] = {
      0x2C, 0x00, 0x00, 0x02, 0x00, 0x01,  0x0B, 0x80, 0x00,  // TH + RH
      0x01, 0x06, 0x81, 0x00,  0x40, 0x40, 0x40, 0x40,  // Mode name
      0x40, 0x40, 0x40, 0x40,  0xF3, 0x08, 0x40, 0x40,  // Req id
      0X40, 0x40, 0x40, 0x40,  0x40, 0x40,
      0x00, 0x00, 0x00 };

char RSP_buf[BUFLEN_3270];                  // Response buffer
int double_up_iac (BYTE *buf, int len);

/*-------------------------------------------------------------------*/
/* Print VTAM connected or disconnected message.                     */
/*-------------------------------------------------------------------*/
void connect_message(int sfd, int na, int flag) {
   int rc;
   struct sockaddr_in client;
   socklen_t namelen;
   char *ipaddr;
   char msgtext[256];
   if (!sfd)
      return;

   namelen = sizeof(client);
   rc = getpeername (sfd, (struct sockaddr *)&client, &namelen);
   ipaddr = inet_ntoa(client.sin_addr);
   if (flag == 0)
      sprintf(msgtext, "%s:%d VTAM CONNECTION ACCEPTED - NETWORK NODE= %4.4X",
              ipaddr, (int)ntohs(client.sin_port), na);
   else
      sprintf(msgtext, "%s:%d VTAM CONNECTION TERMINATED.",
              ipaddr, (int)ntohs(client.sin_port));
  // logmsg( _("%s\n"), msgtext);

   write(sfd, msgtext, strlen(msgtext));
   write(sfd, "\r\n", 2);
}

/*-------------------------------------------------------------------*/
/* Process FID2 PIU.                                                 */
/*-------------------------------------------------------------------*/
//
//    <-------------------------- PIU -------------------------->
//    <------------ TH -----------> <---- RH ----> <---- RU ---->
//     0    1    2    3    4    5    6    7    8    9         n
//   |----+----+----+----+----+----|----+----+----|----+---//----|
//   |FID2|resv|DAF |OAF | seq nr. |RH_0|RH_1|RH_2|RU_0...    ...|
//   |----+----+----+----+----+----|----+----+----|----+---//----|
//
int proc_PIU (char BLU_buf[], int Pptr, int Blen, int Fcntl) {
   // PIU-buf[Pptr] must point to byte 0 of the TH.
   // Fcntl: RR / IFRAME / IFRAME + Cpoll
   BYTE *ru_ptr;                       // ???
   BYTE  Dbuf[BUFLEN_3270];            // Data buffer
   int   RUlen = 16;                   // RU response length
   int   i;
   register char *s;

   if (debug_reg & 0x20) {             // Debug ?
      if ((Fcntl & 0x0F) == RR) {      // RR format ?
         fprintf(trace, "PIU0=> Supervisory RR received. \n");
      } else {                         // Must be a IFRAME with PIU
         fprintf(trace, "PIU0=> Iframe received: Pptr=%2d, Blen=%2d, Fcntl=0x%02X \n", Pptr, Blen, Fcntl);
         fprintf(trace, "PIU0=>[%d]: ", Pptr );
         for (s = (char *) &BLU_buf[Pptr], i = 0; i < (Blen - Pptr); ++i, ++s)
            fprintf(trace, "%02X ", (int) *s & 0xFF);
         fprintf(trace, "\n");
      }
   }

   //================================================================
   // RR format received
   //================================================================
   if ((Fcntl & 0x0F) == RR) {         // Only a RR ?
      if (Rsp_buf == EMPTY) {          // Empty ?
         if (ca->inpbufl > 0) {     
            /* Copy 3270 input to Response buffer after TH and RH */
            for (int j = 9; j < ca->inpbufl + 9; j++)
               RSP_buf[j] = ca->inpbuf[j-9];

            /* Construct 6 byte FID2 TH */
            RSP_buf[FD2_TH_0]    = 0x2E;     // FID2
            RSP_buf[FD2_TH_1]    = 0x00;     // Reserved
            RSP_buf[FD2_TH_daf]  = ca->tso_addr1;   //  daf
            RSP_buf[FD2_TH_oaf]  = ca->lu_addr1;   //  oaf
            RSP_buf[FD2_TH_scf0] = 0x00;  // seq #
            RSP_buf[FD2_TH_scf1] = 0x00;
            make_seq(ca, RSP_buf);

            /* Construct 3 byte FID2 RH */
            RSP_buf[FD2_RH_0] =  0x00;
            RSP_buf[FD2_RH_0] |= 0x03;           // Indicate this is first and last in chain
            RSP_buf[FD2_RH_1] =  0x90;
            RSP_buf[FD2_RH_2] =  0x20;           // Indicate Change Direction

            Plen= 3 + 6 + ca->inpbufl;           // Update PIU length
            ca->inpbufl = 0;

            Pptr = 3;                            // Reset ptr to begin of BLU buffer
 
            memcpy(&BLU_buf[Pptr], &RSP_buf[FD2_TH_0], Plen);

            if (debug_reg & 0x20) {              // Debug ?
               fprintf(trace, "PIU4<= 3270 Data Stream<=[%d]: ",Plen);
               for (i = 0; i < Plen; i++)
                  fprintf(trace, "%02X ", (int) BLU_buf[Pptr+i] & 0xFF);
               fprintf(trace, "\n");
            }

            /* Send 3270 data response to host */
            Rsp_buf = EMPTY;
            return(Plen);                 // Send PIU to host
         } else {
            return 0;
         }
      } else {                         // Response buffer is filled with a SNA cmd resp.

         // Clear BLU buffer, update RUlen & buffer content.
         Pptr = 3;                     // Reset ptr to begin of BLU buffer
         for (int j = Pptr; j++, j < 65536;)
            BLU_buf[j] = 0x00;
         memcpy(&BLU_buf[Pptr + FD2_TH_0], &RSP_buf[FD2_TH_0], Plen);

         /* Send response to host */
         if (debug_reg & 0x20) {
            fprintf(trace, "PIU3<=[%d]: ", Pptr );
            for (s = (char *) &BLU_buf[Pptr], i = 0; i < (Plen); ++i, ++s)
               fprintf(trace, "%02X ", (int) *s & 0xFF);
            fprintf(trace, "\n");
         }
         Rsp_buf = EMPTY;
         return(Plen);                 // Send PIU to host
      }

      /***********************/
      /*** INITSELF Req    ***/
      /***********************/
      if ((ca->sfd > 0) && (ca->bindflag) && (ca->initselfflag == 0) && (Rsp_buf == EMPTY)) {
         memcpy(&BLU_buf[Pptr + FD2_TH_0], F2_INITSELF_Req, sizeof(F2_INITSELF_Req));   // TEMP !!!

         // Very dirty, but it works for now...
         BLU_buf[Pptr + FD2_TH_oaf] = last_lu;
         Plen = sizeof(F2_INITSELF_Req);

         if (debug_reg & 0x20) {
            fprintf(trace, "PIU4<=[%d]: ", Pptr );
            for (s = (char *) &BLU_buf[Pptr], i = 0; i < (Plen); ++i, ++s)
               fprintf(trace, "%02X ", (int) *s & 0xFF);
            fprintf(trace, "\n");
         }

         ca->initselfflag = 1;         // Flag INITSELF send.
         return(Plen);

      } else {
         return(0);                    // Send RR back, no work
      }
   }

   //================================================================
   // Iframe received with an PIU
   //================================================================
   if ((Fcntl & 0x01) == IFRAME) {

      /**********************************************************/
      /*** PROCESS IFRAME as SNA cmd or as TN3270 DATA STREAM ***/
      /**********************************************************/
      if ((BLU_buf[Pptr + FD2_RH_0] & 0x80) == 0x80) {
         // Disregard if this is a response PIU.
         return 0;
      }

      /* If type=data, and DAF matches up, and socket exists, do: */
      if (((BLU_buf[Pptr + FD2_RH_0] & (unsigned char)0xFC) == 0x00) &&
//         (BLU_buf[Pptr + FD2_TH_daf] == ca->lu_addr1 &&
            ca->sfd > 0) {
         if (debug_reg & 0x20) {
            fprintf(trace, "PIU2=>[%d]: ", Pptr);
            for (s = (char *) &BLU_buf[Pptr], i = 0; i < (Blen - Pptr); ++i, ++s)
               fprintf(trace, "%02X ", (int) *s & 0xFF);
            fprintf(trace, "\n");
         }

         // Get RUlen by searching for x'470F7E'. (CRC + EFlag)
         // and copy data to the Dbuf.
         i = 0;
         while (!((BLU_buf[Pptr + FD2_RU_0 + i+0] == 0x47) &&
                (BLU_buf[Pptr + FD2_RU_0 + i+1] == 0x0F) &&
                (BLU_buf[Pptr + FD2_RU_0 + i+2] == 0x7E))) {
            Dbuf[i] = BLU_buf[Pptr + FD2_RU_0 + i];
            i++;
         }
         RUlen = i;

         RUlen = double_up_iac(Dbuf, RUlen);

         if ((BLU_buf[Pptr + FD2_RH_0] & 0x01) == 0x01)  {  // End chain?
            Dbuf[RUlen++] = IAC;
            Dbuf[RUlen++] = EOR_MARK;
         }
         
         if (debug_reg & 0x20) {
            fprintf(trace, "3270=>[%d]: ", RUlen);
            for ( i = 0; i < RUlen; i++)
               fprintf(trace, "%02X ", (int) Dbuf[i] & 0xFF);
            fprintf(trace, "\n");
         }
         
         //************************************************************
         send_packet (ca->sfd, (BYTE *) Dbuf, RUlen, "3270 Data");
         //************************************************************
      }

      if ((BLU_buf[Pptr + FD2_RH_1] & 0xF0) != 0x80)       // Disregard if not DR1 requested
         return 0;

      /* Construct 6 byte FID2 TH */
      RSP_buf[FD2_TH_0]    = BLU_buf[Pptr + FD2_TH_0];     // FID2
      RSP_buf[FD2_TH_1]    = BLU_buf[Pptr + FD2_TH_1];     // Reserved
      RSP_buf[FD2_TH_daf]  = BLU_buf[Pptr + FD2_TH_oaf];   // oaf -> daf
      RSP_buf[FD2_TH_oaf]  = BLU_buf[Pptr + FD2_TH_daf];   // daf -> oaf
      RSP_buf[FD2_TH_scf0] = BLU_buf[Pptr + FD2_TH_scf0];  // seq #
      RSP_buf[FD2_TH_scf1] = BLU_buf[Pptr + FD2_TH_scf1];

      /* Construct 3 byte FID2 RH */
      RSP_buf[FD2_RH_0] =  BLU_buf[Pptr + FD2_RH_0];
      RSP_buf[FD2_RH_0] |= 0x83;           // Indicate this is a Response
      RSP_buf[FD2_RH_1] =  BLU_buf[Pptr + FD2_RH_1] & 0xEF;  // +Rsp
      RSP_buf[FD2_RH_2] =  0x00;

      Plen = 9;
      Rsp_buf = FILLED;

      /***********************/
      /*** ACTPU (PU)      ***/
      /***********************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x11 && BLU_buf[Pptr + FD2_RU_1] == 0x01) {
         /* Save daf as our own net addr */
         ca->pu_addr0 = 0x00;
         ca->pu_addr1 = BLU_buf[Pptr + FD2_TH_daf];
         // Copy +ACTPU to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_ACTPU_Rsp, sizeof(F2_ACTPU_Rsp));

         Plen = 6 + 3 + sizeof(F2_ACTPU_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }

      /***********************/
      /*** ACTLU           ***/
      /***********************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x0D) {
         /* Save daf as our own net addr */
         ca->lu_addr0 = 0x00;
         ca->lu_addr1 = BLU_buf[Pptr + FD2_TH_daf];
         last_lu      = BLU_buf[Pptr + FD2_TH_daf];   // Dirty
         /* Save oaf as our sscp net addr */
         ca->sscp_addr0 = 0x00;
         ca->sscp_addr1 = BLU_buf[Pptr + FD2_TH_oaf];
         /*            */
         ca->lu_sscp_seqn = 0;
         ca->bindflag = 0;
         ca->initselfflag = 0;
         // Copy +ACTLU to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_ACTLU_Rsp, sizeof(F2_ACTLU_Rsp));

         Plen = 6 + 3 + sizeof(F2_ACTLU_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }

      /************************************/
      /*** 01XXXX Network Services (NS) ***/
      /************************************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x01) {
         ru_ptr[RUlen++] = BLU_buf[Pptr + FD2_RU_1];
         ru_ptr[RUlen++] = BLU_buf[Pptr + FD2_RU_2];
      }

      /*** ASSIGN NETW ADDR (ANA) ***/
      if (!memcmp(&RSP_buf[FD2_RU_0], R010219, 3) && ca->sfd > 0) {
   //      if (!ca->is_3270)
   //         connect_message(ca->sfd, (BLU_buf[20] << 8) + BLU_buf[21], 0);
      }

      /*** DACTLU or ABCONN ***/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x0E || !memcmp(&BLU_buf[Pptr + FD2_RU_0], R01020F, 3)) {
   //      if (!ca->is_3270)
   //         connect_message(ca->sfd, 0, 1);
         ca->hangup = 1;
      }

      /***********************/
      /*** BIND            ***/
      /***********************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x31) {
         /* Save oaf from BIND request */
         ca->tso_addr1 = BLU_buf[Pptr + FD2_TH_oaf];
         ca->lu_lu_seqn = 0;
         ca->bindflag = 1;
         // Copy +BIND to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_BIND_Rsp, sizeof(F2_BIND_Rsp));

         Plen = 6 + 3 + sizeof(F2_BIND_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }

      /*******************************/
      /*** SDT (Start Data Traffic ***/
      /*******************************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0xA0) {
         /* Save oaf from BIND request */
         ca->tso_addr1 = BLU_buf[Pptr + FD2_TH_oaf];
         ca->lu_lu_seqn = 0;
         // Copy +SDT to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_SDT_Rsp, sizeof(F2_SDT_Rsp));

         Plen = 6 + 3 + sizeof(F2_SDT_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }

      /*******************************/
      /*** CLEAR                   ***/
      /*******************************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0xA1) {
         /* Save oaf from BIND request */
         ca->tso_addr1 = BLU_buf[Pptr + FD2_TH_oaf];
         ca->lu_lu_seqn = 0;
         ca->ncpa_sscp_seqn = 0;                 // Reset sequence number
         // Copy +CLEAR to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_CLEAR_Rsp, sizeof(F2_CLEAR_Rsp));

         Plen = 6 + 3 + sizeof(F2_CLEAR_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }
      /*******************************/
      /*** DACTLU                  ***/
      /*******************************/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x0E) {
         /* Save oaf from BIND request */
         ca->tso_addr1 = BLU_buf[Pptr + FD2_TH_oaf];
         ca->lu_lu_seqn = 0;
         // Copy +CLEAR to RU.
         memcpy(&RSP_buf[FD2_RU_0], F2_DACTLU_Rsp, sizeof(F2_DACTLU_Rsp));

         Plen = 6 + 3 + sizeof(F2_DACTLU_Rsp);    // Set PIU length Th+Rh+Ru
         Rsp_buf = FILLED;
      }

      /*** UNBIND & Normal end of session ***/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x32 && BLU_buf[Pptr + FD2_RU_1] != 0x02) {
         ca->bindflag = 0;
      }
#if 0
      /*** UNBIND ****/
      if (BLU_buf[Pptr + FD2_RU_0] == 0x32 && BLU_buf[Pptr + FD2_RU_1] == 0x01 && ca->sfd > 0) {
         close_socket(ca->sfd);
         ca->sfd=-1;
      }
#endif

      /******************************************************************/
      /* End assembly: copy TH + RH +Rsp + RU to PIU buffer             */
      /******************************************************************/
      Pptr = 3;
      fprintf(trace, "PIU3 Fcntl: %02X \n ", Fcntl );
      if (Fcntl & 0x10) {              // Poll bit on ?
         memcpy(&BLU_buf[Pptr + FD2_TH_0], &RSP_buf[FD2_TH_0], Plen);

         if (debug_reg & 0x20) {
            fprintf(trace, "PIU3<=[%d]: ", Pptr );
            for (s = (char *) &BLU_buf[Pptr], i = 0; i < (Plen); ++i, ++s)
               fprintf(trace, "%02X ", (int) *s & 0xFF);
            fprintf(trace, "\n");
         }

         /* Send response to host */
         Rsp_buf = EMPTY;
         return (Plen);
      } else {
         // No poll bit on; Resp_buf must wait...
         return 0;
      }
   }
}
//#####################################################################


/*-------------------------------------------------------------------*/
/* Subroutine to create unique PIU sequence numbers.                 */
/*-------------------------------------------------------------------*/
void make_seq (COMMADPT * ca, BYTE * bufptr) {
        bufptr[4] = (unsigned char)(++ca->ncpa_sscp_seqn >> 8) & 0xff;
        bufptr[5] = (unsigned char)(  ca->ncpa_sscp_seqn     ) & 0xff;
}

/*-------------------------------------------------------------------*/
/* Subroutine to double up any IAC bytes in the data stream.         */
/* Returns the new length after inserting extra IAC bytes.           */
/*-------------------------------------------------------------------*/
int double_up_iac (BYTE *Dbuf, int len) {
   int m, n, x, newlen;

   /* Count the number of IAC bytes in the data */
   for (x = 0, n = 0; n < len; n++)
      if (Dbuf[n] == IAC) x++;

   /* Exit if nothing to do */
   if (x == 0) return len;

   /* Insert extra IAC bytes backwards from the end of the buffer */
   newlen = len + x;
   if (debug_reg & 0x20) {
      fprintf(trace, "CC1: %d IAC bytes added, newlen = %d\n",
                      x, newlen);
   }
   for (n = newlen, m = len; n > m; ) {
      Dbuf[--n] = Dbuf[--m];
      if (Dbuf[n] == IAC) Dbuf[--n] = IAC;
   }
   return newlen;
}  /* End of function double_up_iac */


unsigned char host_to_guest (unsigned char byte)
{
    return (unsigned char)codepage_conv->h2g[(unsigned int)byte];
}


BYTE* prt_host_to_guest( const BYTE *psinbuf, BYTE *psoutbuf, const u_int ilength )
{
   u_int   count;
   int     pad = FALSE;

   for (count = 0; count < ilength; count++ ) {
      if ( !pad && psinbuf[count] == '\0' )
          pad = TRUE;
      if ( !pad ) {
          psoutbuf[count] = isprint      (psinbuf[count]) ?
                            host_to_guest(psinbuf[count]) :
                            host_to_guest('.');
      } else {
          psoutbuf[count] = host_to_guest(' ');
      }
   }
   return psoutbuf;
}


/*-------------------------------------------------------------------*/
/* Write "n" bytes to a descriptor.                                  */
/* Use in place of write() when fd is a stream socket.               */
/*-------------------------------------------------------------------*/
int write_socket( int fd, const void *_ptr, int nbytes )
{
const char *ptr;
int  nleft, nwritten;

   ptr   = _ptr;               /* point to data to be written       */
   nleft = nbytes;             /* number of bytes to be written     */

   while (nleft > 0)           /* while bytes remain to be written  */
   {
        nwritten = write( fd, ptr, nleft );

        if (nwritten <= 0)
            return nwritten;    /* error, return <= 0 */

        ptr   += nwritten;      /* bump to next o/p buffer location  */
        nleft -= nwritten;      /* fix remaining bytes to be written */

    } /* end of do while */

    return (nbytes - nleft);    /* return number of bytes written */

} /* end of write_socket */

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO SEND A DATA PACKET TO THE CLIENT                    */
/*-------------------------------------------------------------------*/
int send_packet(int csock, BYTE *buf, int len, char *caption)
{
int     rc;                             /* Return code               */

    rc = send (csock, buf, len, 0);

    if (rc < 0) {
        printf("\nsend to client failed");
        return -1;
    } /* end if(rc) */

    return 0;

} /* end function send_packet */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A DATA PACKET FROM THE CLIENT               */
/* This subroutine receives bytes from the client.  It stops when    */
/* the receive buffer is full, or when the last two bytes received   */
/* consist of the IAC character followed by a specified delimiter.   */
/* If zero bytes are received, this means the client has closed the  */
/* connection, and this is treated as an error.                      */
/* Input:                                                            */
/*      csock is the socket number                                   */
/*      buf points to area to receive data                           */
/*      reqlen is the number of bytes requested                      */
/*      delim is the delimiter character (0=no delimiter)            */
/* Output:                                                           */
/*      buf is updated with data received                            */
/*      The return value is the number of bytes received, or         */
/*      -1 if an error occurred.                                     */
/*-------------------------------------------------------------------*/
static int
recv_packet (int csock, BYTE *buf, int reqlen, BYTE delim)
{
int     rc=0;                           /* Return code               */
int     rcvlen=0;                       /* Length of data received   */

    while (rcvlen < reqlen) {

        rc = recv (csock, buf + rcvlen, reqlen - rcvlen, 0);

        if (rc < 0) {
           // WRMSG(HHC01034, "E", "recv()", strerror(HSO_errno));
            return -1;
        }

        if (rc == 0) {
            //TNSDEBUG1("console: DBG004: Connection closed by client\n");
            return -1;
        }

        rcvlen += rc;

        if (delim != '\0' && rcvlen >= 2
            && buf[rcvlen-2] == IAC && buf[rcvlen-1] == delim)
            break;
    }

    //TNSDEBUG2("console: DBG005: Packet received length=%d\n", rcvlen);
    //packet_trace (buf, rcvlen);

    return rcvlen;

} /* end function recv_packet */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A PACKET AND COMPARE WITH EXPECTED VALUE    */
/*-------------------------------------------------------------------*/
static int
expect (int csock, BYTE *expected, int len, char *caption)
{
int     rc;                             /* Return code               */
BYTE    buf[512];                       /* Receive buffer            */

#if defined( OPTION_MVS_TELNET_WORKAROUND )

  /* TCP/IP for MVS returns the server sequence rather then the
     client sequence during bin negotiation.   Jan Jaeger, 19/06/00  */

  static BYTE do_bin[] = { IAC, DO, BINARY, IAC, WILL, BINARY };
  static BYTE will_bin[] = { IAC, WILL, BINARY, IAC, DO, BINARY };

#endif // defined( OPTION_MVS_TELNET_WORKAROUND )

    //UNREFERENCED(caption);

    rc = recv_packet (csock, buf, len, 0);
    if (rc < 0)
        return -1;

#if defined( OPTION_MVS_TELNET_WORKAROUND )
    /* BYPASS TCP/IP FOR MVS WHICH DOES NOT COMPLY TO RFC1576 */
    if (1
        && memcmp(buf, expected, len) != 0
        && !(len == sizeof(will_bin)
        && memcmp(expected, will_bin, len) == 0
        && memcmp(buf, do_bin, len) == 0)
    )
#else
    if (memcmp(buf, expected, len) != 0)
#endif // defined( OPTION_MVS_TELNET_WORKAROUND )
    {
        //TNSDEBUG2("console: DBG006: Expected %s\n", caption);
        return -1;
    }
   // TNSDEBUG2("console: DBG007: Received %s\n", caption);

    return 0;

} /* end function expect */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO NEGOTIATE TELNET PARAMETERS                         */
/* This subroutine negotiates the terminal type with the client      */
/* and uses the terminal type to determine whether the client        */
/* is to be supported as a 3270 display console or as a 1052/3215    */
/* printer-keyboard console.                                         */
/*                                                                   */
/* Valid display terminal types are "IBM-NNNN", "IBM-NNNN-M", and    */
/* "IBM-NNNN-M-E", where NNNN is 3270, 3277, 3278, 3279, 3178, 3179, */
/* or 3180, M indicates the screen size (2=25x80, 3=32x80, 4=43x80,  */
/* 5=27x132, X=determined by Read Partition Query command), and      */
/* -E is an optional suffix indicating that the terminal supports    */
/* extended attributes. Displays are negotiated into tn3270 mode.    */
/* An optional device number suffix (example: IBM-3270@01F) may      */
/* be specified to request allocation to a specific device number.   */
/* Valid 3270 printer type is "IBM-3287-1"                           */
/*                                                                   */
/* Terminal types whose first four characters are not "IBM-" are     */
/* handled as printer-keyboard consoles using telnet line mode.      */
/*                                                                   */
/* Input:                                                            */
/*      csock   Socket number for client connection                  */
/* Output:                                                           */
/*      class   D=3270 display console, K=printer-keyboard console   */
/*              P=3270 printer                                       */
/*      model   3270 model indicator (2,3,4,5,X)                     */
/*      extatr  3270 extended attributes (Y,N)                       */
/*      devn    Requested device number, or FFFF=any device number   */
/* Return value:                                                     */
/*      0=negotiation successful, -1=negotiation error               */
/*-------------------------------------------------------------------*/
int negotiate(int csock, BYTE *class, BYTE *model, BYTE *extatr, U16 *devn,char *group)
{
int    rc;                              /* Return code               */
char  *termtype;                        /* Pointer to terminal type  */
char  *s;                               /* String pointer            */
BYTE   c;                               /* Trailing character        */
U16    devnum;                          /* Requested device number   */
BYTE   buf[512];                        /* Telnet negotiation buffer */
static BYTE do_term[] = { IAC, DO, TERMINAL_TYPE };
static BYTE will_term[] = { IAC, WILL, TERMINAL_TYPE };
static BYTE req_type[] = { IAC, SB, TERMINAL_TYPE, SEND, IAC, SE };
static BYTE type_is[] = { IAC, SB, TERMINAL_TYPE, IS };
static BYTE do_eor[] = { IAC, DO, EOR, IAC, WILL, EOR };
static BYTE will_eor[] = { IAC, WILL, EOR, IAC, DO, EOR };
static BYTE do_bin[] = { IAC, DO, BINARY, IAC, WILL, BINARY };
static BYTE will_bin[] = { IAC, WILL, BINARY, IAC, DO, BINARY };
#if 0
static BYTE do_tmark[] = { IAC, DO, TIMING_MARK };
static BYTE will_tmark[] = { IAC, WILL, TIMING_MARK };
static BYTE wont_sga[] = { IAC, WONT, SUPPRESS_GA };
static BYTE dont_sga[] = { IAC, DONT, SUPPRESS_GA };
#endif
static BYTE wont_echo[] = { IAC, WONT, ECHO_OPTION };
static BYTE dont_echo[] = { IAC, DONT, ECHO_OPTION };
static BYTE will_naws[] = { IAC, WILL, NAWS };

    /* Perform terminal-type negotiation */
    rc = send_packet (csock, do_term, sizeof(do_term),
                        "IAC DO TERMINAL_TYPE");
    if (rc < 0) return -1;

    rc = expect (csock, will_term, sizeof(will_term),
                        "IAC WILL TERMINAL_TYPE");
    if (rc < 0) return -1;

    /* Request terminal type */
    rc = send_packet (csock, req_type, sizeof(req_type),
                        "IAC SB TERMINAL_TYPE SEND IAC SE");
    if (rc < 0) return -1;

    rc = recv_packet (csock, buf, sizeof(buf)-2, SE);
    if (rc < 0) return -1;

    /* Ignore Negotiate About Window Size */
    if (rc >= (int)sizeof(will_naws) &&
        memcmp (buf, will_naws, sizeof(will_naws)) == 0)
    {
        memmove(buf, &buf[sizeof(will_naws)], (rc - sizeof(will_naws)));
        rc -= sizeof(will_naws);
    }

    if (rc < (int)(sizeof(type_is) + 2)
        || memcmp(buf, type_is, sizeof(type_is)) != 0
        || buf[rc-2] != IAC || buf[rc-1] != SE) {
  //      TNSDEBUG2("console: DBG008: Expected IAC SB TERMINAL_TYPE IS\n");
        return -1;
    }
    buf[rc-2] = '\0';
    termtype = (char *)(buf + sizeof(type_is));
 //   TNSDEBUG2("console: DBG009: Received IAC SB TERMINAL_TYPE IS %s IAC SE\n",
         //   termtype);

    /* Check terminal type string for device name suffix */
    s = strchr (termtype, '@');
    if(s!=NULL)
    {
        if(strlen(s)<16)
        {
            strlcpy(group,&s[1],16);
        }
    }
    else
    {
        group[0]=0;
    }

    if (s != NULL && sscanf (s, "@%hx%c", &devnum,&c) == 1)
    {
        *devn = devnum;
        group[0]=0;
    }
    else
    {
        *devn = 0xFFFF;
    }

    /* Test for non-display terminal type */
    if (memcmp(termtype, "IBM-", 4) != 0)
    {
#if 0
        /* Perform line mode negotiation */
        rc = send_packet (csock, do_tmark, sizeof(do_tmark),
                            "IAC DO TIMING_MARK");
        if (rc < 0) return -1;

        rc = expect (csock, will_tmark, sizeof(will_tmark),
                            "IAC WILL TIMING_MARK");
        if (rc < 0) return 0;

        rc = send_packet (csock, wont_sga, sizeof(wont_sga),
                            "IAC WONT SUPPRESS_GA");
        if (rc < 0) return -1;

        rc = expect (csock, dont_sga, sizeof(dont_sga),
                            "IAC DONT SUPPRESS_GA");
        if (rc < 0) return -1;
#endif

        if (memcmp(termtype, "ANSI", 4) == 0)
        {
            rc = send_packet (csock, wont_echo, sizeof(wont_echo),
                                "IAC WONT ECHO");
            if (rc < 0) return -1;

            rc = expect (csock, dont_echo, sizeof(dont_echo),
                                "IAC DONT ECHO");
            if (rc < 0) return -1;
        }

        /* Return printer-keyboard terminal class */
        *class = 'K';
        *model = '-';
        *extatr = '-';
        return 0;
    }

    /* Determine display terminal model */
    if (memcmp(termtype+4,"DYNAMIC",7) == 0) {
        *model = 'X';
        *extatr = 'Y';
    } else {
        if (!(memcmp(termtype+4, "3277", 4) == 0
              || memcmp(termtype+4, "3270", 4) == 0
              || memcmp(termtype+4, "3178", 4) == 0
              || memcmp(termtype+4, "3278", 4) == 0
              || memcmp(termtype+4, "3179", 4) == 0
              || memcmp(termtype+4, "3180", 4) == 0
              || memcmp(termtype+4, "3287", 4) == 0
              || memcmp(termtype+4, "3279", 4) == 0))
            return -1;

        *model = '2';
        *extatr = 'N';

        if (termtype[8]=='-') {
            if (termtype[9] < '1' || termtype[9] > '5')
                return -1;
            *model = termtype[9];
            if (memcmp(termtype+4, "328",3) == 0) *model = '2';
            if (memcmp(termtype+10, "-E", 2) == 0)
                *extatr = 'Y';
        }
    }

    /* Perform end-of-record negotiation */
    rc = send_packet (csock, do_eor, sizeof(do_eor),
                        "IAC DO EOR IAC WILL EOR");
    if (rc < 0) return -1;

    rc = expect (csock, will_eor, sizeof(will_eor),
                        "IAC WILL EOR IAC DO EOR");
    if (rc < 0) return -1;

    /* Perform binary negotiation */
    rc = send_packet (csock, do_bin, sizeof(do_bin),
                        "IAC DO BINARY IAC WILL BINARY");
    if (rc < 0) return -1;

    rc = expect (csock, will_bin, sizeof(will_bin),
                        "IAC WILL BINARY IAC DO BINARY");
    if (rc < 0) return -1;

    /* Return display terminal class */
    if (memcmp(termtype+4,"3287",4)==0) *class='P';
    else *class = 'D';
    return 0;

} /* end function negotiate */


/*-------------------------------------------------------------------*/
/* NEW CLIENT CONNECTION                                             */
/*-------------------------------------------------------------------*/
static int
connect_client (int *csockp)
/* returns 1 if 3270, else 0 */
{
int                     rc;             /* Return code               */
size_t                  len;            /* Data length               */
int                     csock;          /* Socket for conversation   */
struct sockaddr_in      client;         /* Client address structure  */
socklen_t               namelen;        /* Length of client structure*/
char                   *clientip;       /* Addr of client ip address */
U16                     devnum;         /* Requested device number   */
BYTE                    class;          /* D=3270, P=3287, K=3215/1052 */
BYTE                    model;          /* 3270 model (2,3,4,5,X)    */
BYTE                    extended;       /* Extended attributes (Y,N) */
char                    buf[256];       /* Message buffer            */
char                    conmsg[256];    /* Connection message        */
char                    devmsg[25];     /* Device message            */
char                    hostmsg[256];   /* Host ID message           */
char                    num_procs[16];  /* #of processors string     */
char                    group[16];      /* Console group             */

    /* Load the socket address from the thread parameter */
    csock = *csockp;

    /* Obtain the client's IP address */
    namelen = sizeof(client);
    rc = getpeername (csock, (struct sockaddr *)&client, &namelen);

    /* Log the client's IP address and hostname */
    clientip = strdup(inet_ntoa(client.sin_addr));


    /* Negotiate telnet parameters */
    rc = negotiate (csock, &class, &model, &extended, &devnum, group);
    if (rc != 0)
    {
        close (csock);
        if (clientip) free(clientip);
        return 0;
    }

    /* Build connection message for client */

 //   if ( cons_hostinfo.num_procs > 1 )
  //      snprintf( num_procs, sizeof(num_procs)-1, "MP=%d", cons_hostinfo.num_procs );
  //  else
        strlcpy( num_procs, "UP", sizeof(num_procs) );

/*    snprintf
    (
        hostmsg, sizeof(hostmsg),

        "running on %s (%s-%s.%s %s %s)"

        ,cons_hostinfo.nodename
        ,cons_hostinfo.sysname
        ,cons_hostinfo.release
        ,cons_hostinfo.version
        ,cons_hostinfo.machine
        ,num_procs
    );
    */

//        snprintf (conmsg, sizeof(conmsg)-1,
//                "3705 version %s built on %s %s",
//               VERSION, __DATE__, __TIME__);
        snprintf (devmsg, sizeof(devmsg)-1, "Connected to device %4.4X",
                  0);


    //WRMSG(HHC01018, "I", 0, 0, clientip, 0x3270);

    /* Send connection message to client */
    if (class != 'K')
    {
        len = snprintf (buf, sizeof(buf)-1,
                    "\xF5\x40\x11\x40\x40\x1D\x60%s"
                    "\x11\xC1\x50\x1D\x60%s"
                    "\x11\xC2\x60\x1D\x60%s",
                    prt_host_to_guest( (BYTE*) conmsg,  (BYTE*) conmsg,  strlen( conmsg  )),
                    prt_host_to_guest( (BYTE*) hostmsg, (BYTE*) hostmsg, strlen( hostmsg )),
                    prt_host_to_guest( (BYTE*) devmsg,  (BYTE*) devmsg,  strlen( devmsg  )));

        if (len < sizeof(buf))
        {
            buf[len++] = IAC;
        }
        else
        {
           // ASSERT(FALSE);
        }

        if (len < sizeof(buf))
        {
            buf[len++] = EOR_MARK;
        }
        else
        {
            //ASSERT(FALSE);
        }
    }
    else
    {
        len = snprintf (buf, sizeof(buf)-1, "%s\r\n%s\r\n%s\r\n",
                        conmsg, hostmsg, devmsg);
    }

    if (class != 'P')  /* do not write connection resp on 3287 */
    {
        rc = send_packet (csock, (BYTE *)buf, (int)len, "CONNECTION RESPONSE");
    }
    return (class == 'D') ? 1 : 0;   /* return 1 if 3270 */
} /* end function connect_client */

 /********************************************************************/
 /* function to read the data from the 3270 terminal                 */
 /********************************************************************/

static void commadpt_read_tty(COMMADPT *ca, BYTE * bfr, int len)
// everything has been tortured to now do 3270 also
{
    BYTE        bfr3[3];
    BYTE        c;
    int i1;
    int eor=0;
  // logdump("RECV",ca->dev, bfr,len);
    /* If there is a complete data record already in the buffer
       then discard it before reading more data
       For TTY, allow data to accumulate until CR is received */

    if (ca->is_3270) {
            if (ca->inpbufl) {
                ca->rlen3270 = 0;
                ca->inpbufl = 0;
            }
        }


    for (i1 = 0; i1 < len; i1++) {
        c = (unsigned char) bfr[i1];

        if (ca->telnet_opt) {
            ca->telnet_opt = 0;
            printf("Tenet_opt\n\r");
            bfr3[0] = 0xff;  /* IAC */
            /* set won't/don't for all received commands */
            bfr3[1] = (ca->telnet_cmd == 0xfd) ? 0xfc : 0xfe;
            bfr3[2] = c;
            if (ca->sfd > 0) {
                write_socket(ca->sfd,bfr3,3);
            }

            continue;
        }
        if (ca->telnet_iac) {
            ca->telnet_iac = 0;

            switch (c) {
            case 0xFB:  /* TELNET WILL option cmd */
            case 0xFD:  /* TELNET DO option cmd */
                ca->telnet_opt = 1;
                ca->telnet_cmd = c;
                break;
            case 0xF4:  /* TELNET interrupt */
                if (!ca->telnet_int) {
                    ca->telnet_int = 1;
                }
                break;
            case EOR_MARK:
                                eor = 1;
                break;
            case 0xFF:  /* IAC IAC */
                        ca->inpbuf[ca->rlen3270++] = 0xFF;
                break;
            }
            continue;
        }
        if (c == 0xFF) {  /* TELNET IAC */

            ca->telnet_iac = 1;
            continue;
        } else {
            ca->telnet_iac = 0;
        }
        if (!ca->is_3270) {
            if (c == 0x0D) // CR in TTY mode ?
                ca->eol_flag = 1;
            c = host_to_guest(c);   // translate ASCII to EBCDIC for tty
        }
        ca->inpbuf[ca->rlen3270++] = c;

    }
    /* received data (rlen3270 > 0) is sufficient for 3270,
       but for TTY, eol_flag must also be set */
       printf("\n");

    if ((ca->eol_flag || ca->is_3270) && ca->rlen3270)
    {
        ca->eol_flag = 0;
        if (ca->is_3270)
        {
            if (eor)
            {
                ca->inpbufl = ca->rlen3270;
                ca->rlen3270 = 0; /* for next msg */
            }
        }
        else
        {
            ca->inpbufl = ca->rlen3270;
            ca->rlen3270 = 0; /* for next msg */
        }
    }
}


/********************************************************************/
/* Thread to handle connection request from 3270 emulators          */
/********************************************************************/
void *TEL_thread(void *arg)
{
    int devnum;                 /* device number copy for convenience*/
    int        sockopt;         /* Used for setsocketoption          */
    int rc;                     /* return code from various rtns     */
//    int ifa;                    /* Interface address                 */
    struct sockaddr_in  sin, *sin2;     /* bind socket address structure     */
    struct ifaddrs *nwaddr, *ifa;      /* interface address structure       */
    char *ipaddr;
    BYTE bfr[256];
    fprintf(stderr, "\nTEL: thread %d started succesfully... \n",syscall(SYS_gettid));

    ca =  malloc(sizeof(COMMADPT));
    
    getifaddrs(&nwaddr);      /* get network address */
    for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name,"lo")) {
           sin2 = (struct sockaddr_in *) ifa->ifa_addr;
           ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
           if (strcmp(ifa->ifa_name,"eth")) break;
        }
    }
    printf("\nTEL: Using network Address %s on %s for 3270 client connections\n",ipaddr,ifa->ifa_name);


    printf("\nca value is %p \n\r",ca);
    /* get a work copy of devnum (for messages) */
    ca->sfd = 0;
    devnum=ca->devnum;

    ca->lfd=socket(AF_INET,SOCK_STREAM,0);


    /* Reuse the address regardless of any */
    /* spurious connection on that port    */
    sockopt=1;
    setsockopt(ca->lfd,SOL_SOCKET,SO_REUSEADDR,(void*)&sockopt,sizeof(sockopt));

    /* Bind the socket */
    sin.sin_family=AF_INET;
    sin.sin_addr.s_addr = inet_addr(ipaddr);
    sin.sin_port=htons(32001);
    rc=bind(ca->lfd,(struct sockaddr *)&sin,sizeof(sin));
    if (rc < 0)
    {
        printf("\nTEL: Socket bind failed\n");
        return NULL;
    }
    /* Start the listen */
    rc = listen(ca->lfd, 10);
    if (rc < 0)
    {
        //WRMSG(HHC01000, "E",SSID_TO_LCSS(ca->dev->ssid),devnum,"listen()",strerror(HSO_errno));
        return NULL;
    }
    //WRMSG(HHC01004, "I", SSID_TO_LCSS(ca->dev->ssid), devnum, ca->lport);
    for (;;)
    {
        ca->sfd = 0;
        ca->sfd=accept(ca->lfd,NULL,0);
        if (ca->sfd < 1)
            continue;
        if  (connect_client(&ca->sfd))
        {
            ca->is_3270 = 1;
        }
        else
        {
            ca->is_3270 = 0;
        }
        fcntl(ca->sfd, F_SETFL, fcntl(ca->sfd, F_GETFL,0) | O_NONBLOCK);
        //socket_set_blocking_mode(ca->sfd,0);  // set to non-blocking mode
        //make_sna_requests4(ca, 0, (ca->is_3270) ? 0x02 : 0x01);   // send REQCONT
        ca->hangup = 0;
        for (;;)
        {
            usleep(50000);
            if (ca->hangup)
                break;
            /* read_socket has changed from 3.04 to 3.06 - we need old way */

            rc=read(ca->sfd,bfr,256-BUFPD);

            if (rc < 0)
            {
                if (0
                    || EAGAIN == errno
                    || EWOULDBLOCK == errno
                )
                {
                    continue;
                }
                break;
            }
            if (rc == 0)
            {
//              make_sna_requests4(ca, 1);   // send REQDISCONT
 //               make_sna_requests5(ca);
                break;
            }

            commadpt_read_tty(ca,bfr,rc);

        }
        close (ca->sfd);
        ca->sfd = 0;
    }

    return NULL;
}


