/* i3705_sdlc.c: IBM 3705 Communication Scanner Type 2 simulator

   Copyright (c) 2021, Henk Stegeman and Edwin Freekenhorst

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

   -----------------------------------------------------------------------------

   SLDC frame
    <-------------------------------- BLU ----------------------------->
   layout:         |   FCntl   |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
   | BFlag | FAddr |Nr|PF|Ns|Ft| ... Iframe ... | Hfcs  | Lfcs  | EFlag |
   +-------+-------+-----------+-------//-------+-------+-------+-------+

   This code module will simulate an SDLC client.
   Only "needed to function" is implemented.
*/

#include "sim_defs.h"
#include "i3705_defs.h"
#include "i3705_sdlc.h"

extern FILE *trace;
extern int8 debug_reg;
extern int8 last_P_Ns;
extern int8 last_S_Ns;

int8 stat_mode = NDM;
int8 new_S_Nr, new_S_Ns;               // Secondairy station frame numbers
int8 rxtx_dir = RX;                    // Rx or Tx flag
int  Blen;                             // Length of received BLU in buffer

char BLU_buf[65536];                   // DLC header + TH + RH + RU + DLC trailer

char proc_BLU(int8 icw_pcf, char tx_data, int j);  // SDLC frame handler
// void proc_BLU(char BLU_buf[], int Blen);   // SDLC frame handler
void proc_frame(char BLU_buf[], int Fptr, int Blen); // Process frame header
int  proc_PIU(char PIU_buf[], int Fptr, int Blen, int Ftype);   // PIU handler
void trace_Fbuf(char BLU_buf[], int Fptr, int Blen, int rxtx_dir);   // Print trace records

//*********************************************************************
//   SDLC frame (BLU) handler
//*********************************************************************
char proc_BLU(int8 icw_pcf, char tx_data, int j) {
   register char *s;
   int temp;
   int Fptr = 0;
   int direction = TX;

   /*
    *  The scanner makes the following calls to the SDLC handler:
    *  char = proc_BLU(int8 icw_pcf, char tx_data, int j);
    *  -----------------------------------------------------
    *  rx_byte        (     0x6    ,    void    , BLU_idx ) (Monitor for flag)
    *  rx_byte        (     0x7    ,    void    , BLU_idx ) (Transfer data)
    *  void           (     0x9    ,    tx_data , BLU_idx ) (Tx data to buffer)
    *  void           (     0xC    ,    void    , BLU_idx ) (Start xmit buff content)
    */
   switch (icw_pcf) {
      case 0x6:                        // Return (received) byte from BLU buffer
      case 0x7:
         return (BLU_buf[j]);
         break;

      case 0x9:                        // Store icw_pdf in BLU buffer
         BLU_buf[j] = tx_data;
         return 0;
         break;

      case 0xC:                        // Process BLU buffer
         // Search for beginning(s) of SDLC frames and call the handler
         Blen = j;                     // Set length of received BLU in buffer
         while (Fptr < Blen) {
            // Start of an SDLC frame found ?
            if ((BLU_buf[Fptr + BFlag] == 0x7E) && (BLU_buf[Fptr + FAddr] == 0xC1)) {
               if (debug_reg & 0x20)
                  fprintf(trace, "LS1:    Calling proc_FRAME: Fptr=%d, Blen=%d \n", Fptr, Blen);
               // ******************************************************************
               proc_frame(BLU_buf, Fptr, Blen);   // Buffer + Frame ptr & BLU length
               // ******************************************************************
            }
            Fptr++;                    // Keep searching for frames till end of buf.
         }
         Fptr = 0;
         return 0;                     // with response in BLU
         break;
   }
}


//*********************************************************************
//   Process incomming SDLC frame(s) and respond accordingly
//*********************************************************************
void proc_frame(char BLU_buf[], int Fptr, int Blen) {
   register char *s;
   int Pptr;                           // Pointer to start of PIU in BLU buffer
   int Plen;                           // Request or Response PIU length

   if (debug_reg & 0x20)
      trace_Fbuf(BLU_buf, Fptr, Blen, TX);     // Print trace records

   switch (BLU_buf[Fptr + FCntl] & 0x03) {
      case UNNUM:
         // *** UNNUMBERED FORMAT ***
         switch (BLU_buf[Fptr + FCntl] & 0xEF) {
            case SNRM:
               stat_mode = NRM;
               if (BLU_buf[Fptr + FCntl] & CPoll) {   // Poll command ?
                  BLU_buf[Fptr + FCntl] = UA + CFinal;   // Set final
               } else {
                  BLU_buf[BFlag] = 0x00;       // No response
               }
               last_P_Ns = 7; last_S_Ns = 7;   // Dirty fix !!
               break;

            case DISC:
               stat_mode = NDM;                // Thats all for today
               if (BLU_buf[FCntl] & CPoll) {   // Poll command ?
                  BLU_buf[FCntl] = UA + CFinal;
               } else {
                  BLU_buf[BFlag] = 0x00;       // No response
               }
               break;

            case UA:
               break;
            case DM:
               break;
            case FRMR:
               break;
            case TEST:
               break;
            case XID:
               break;
            default:
               break;
         }  // End of switch TCntl & 0xEF
         if (debug_reg & 0x20)
            trace_Fbuf(BLU_buf, Fptr, Blen - Fptr, RX); // Print trace records
         break;

      case SUPRV:
         // *** SUPERVISORY FORMAT ***
         switch (BLU_buf[Fptr + FCntl] & 0x0F) {
            case RR:
               // Check if poll bit is on
               if (BLU_buf[Fptr + FCntl] & CPoll) {   // Poll bit set ?
                  Pptr = Fptr + 3;                    // Pptr points to TH0
                  if (debug_reg & 0x20)
                     fprintf(trace, "LS1:[Super]  Calling proc_PIU 1: Pptr=%d Blen=%d Fcntl=0x%02X \n",
                                      Pptr, Blen, BLU_buf[Fptr + FCntl] );
                  // Call PIU handler
                  // ******************************************************************
                  Plen = proc_PIU(BLU_buf, Pptr, Blen, BLU_buf[Fptr + FCntl]);
                  // ******************************************************************
                  // BLU_buf contains a PIU response when Plen > 0
                  Fptr = 0;
                  if (Plen == 0) {    // Check length of returned PIU size
                     // Send RR with final bit on. (No response PIU)
                     BLU_buf[Fptr + Hfcs]  = 0x47;         // Frame Check Sequence Byte 0.
                     BLU_buf[Fptr + Lfcs]  = 0x0F;         // Frame Check Sequence Byte 1.
                     BLU_buf[Fptr + EFlag] = 0x7E;         // Mark end of frame.
                     if (last_P_Ns == 7) new_S_Nr = 0;     // Update N(r)
                        else new_S_Nr = last_P_Ns + 1;
                     BLU_buf[Fptr + FCntl] = RR + (new_S_Nr << 5) + CFinal;
                  } else {
                     // Send Iframe with a PIU
                     BLU_buf[Fptr + Plen + Hfcs]  = 0x47;  // Frame Check Sequence Byte 0.
                     BLU_buf[Fptr + Plen + Lfcs]  = 0x0F;  // Frame Check Sequence Byte 1.
                     BLU_buf[Fptr + Plen + EFlag] = 0x7E;  // Mark end of frame.
                     // Prim Ns + 1 --> Sec Nr
                     if (last_S_Ns == 7) new_S_Ns = 0;     // Update N(s)
                        else new_S_Ns = last_S_Ns + 1;
                     if (last_P_Ns == 7) new_S_Nr = 0;     // Update N(s)
                        else new_S_Nr = last_P_Ns + 1;
                     BLU_buf[Fptr + FCntl] = (new_S_Nr << 5) + CFinal + (new_S_Ns << 1);
                     last_S_Ns = new_S_Ns;                 //
                  }
                  if (debug_reg & 0x20)
                     trace_Fbuf(BLU_buf, Fptr, 6 + Plen, RX);       // Print trace records
               }
               break;

            case RNR:
               break;
         }
         break;

      case IFRAME:                     // ...00 and
      case IFRAME + 0x02:              // ...10 are both I-frames
         last_P_Ns = Ns;
         // *** INFORMATIONAL FRAME ***
         Pptr = Fptr + 3;           // Points to TH0
         if (debug_reg & 0x20)
            fprintf(trace, "LS1:[Iframe] Calling proc_PIU 2: Pptr=%d Blen=%d Fcntl=0x%02X \n",
                            Pptr, Blen, BLU_buf[Fptr + FCntl] );
         // Call PIU handler
         // ******************************************************************
         Plen = proc_PIU(BLU_buf, Pptr, Blen, BLU_buf[Fptr + FCntl]);
         // ******************************************************************
         // Check if poll bit is on. If yes: send response
         if (BLU_buf[Fptr + FCntl] & CPoll) {
            // Poll bit on. Send a sdlc response with final bit.
            Fptr = 0;                                // Reset frame pointer
            // BLU_buf contains NO response when Plen = 0
            if (Plen == 0) {    // Check length of returned PIU size
               // Send RR to FEP with final bit on. (No response PIU)
               BLU_buf[Fptr + EFlag] = 0x7E;         // Mark end of frame.
               if (last_P_Ns == 7) new_S_Nr = 0;     // Update N(r)
                  else new_S_Nr = last_P_Ns + 1;
               BLU_buf[Fptr + FCntl] = RR + (new_S_Nr << 5) + CFinal;
            } else {
               // Send Iframe to FEP with final bit on. (With response PIU)
               BLU_buf[Fptr + Plen + EFlag] = 0x7E;  // Mark end of frame.
               // Prim Ns + 1 --> Sec Nr
               if (last_S_Ns == 7) new_S_Ns = 0;     // Update N(s)
                  else new_S_Ns = last_S_Ns + 1;
               if (last_P_Ns == 7) new_S_Nr = 0;     // Update N(s)
                  else new_S_Nr = last_P_Ns + 1;
               BLU_buf[Fptr + FCntl] = (new_S_Nr << 5) + CFinal + (new_S_Ns << 1);
               last_S_Ns = new_S_Ns;                 //
            }
         if (debug_reg & 0x20)
            trace_Fbuf(BLU_buf, Fptr, 6 + Plen, RX);
         }
         break;                                         // Next please

   }  // End of switch (rxtx_Fbuf[FCntl] & 0x03)
}

//*********************************************************************
//   Print trace records of frame buffer (Fbuf)
//*********************************************************************
void trace_Fbuf(char BLU_buf[], int Fptr, int Flen, int rxtx_dir) {
   register char *s;
   int i;

   if (rxtx_dir == TX)
      fprintf(trace, "LS1: => ");   // 3705 -> client
   else
      fprintf(trace, "LS1: <= ");   // 3705 <- client

   switch (BLU_buf[Fptr + FCntl] & 0x03) {
      case UNNUM:
         // *** UNNUMBERED FORMAT ***
         switch (BLU_buf[Fptr + FCntl] & 0xEF) {
            case SNRM:
               fprintf(trace, "SNRM - PF=%d \n", PF);
               break;

            case DISC:
               fprintf(trace, "DISC - PF=%d \n", PF);
               break;

            case UA:
               fprintf(trace, "UA - PF=%d \n", PF);
               break;

            case DM:
               fprintf(trace, "DM - PF=%d \n", PF);
               break;

            case FRMR:
               fprintf(trace, "FRMR - PF=%d, TEXT=", PF);
               break;

            case TEST:
               fprintf(trace, "TEST - PF=%d \n", PF);
               break;

            case XID:
               fprintf(trace, "XID - PF=%d \n", PF);
               break;

            default:
               fprintf(trace, "ILLEGAL - ");
               for (s = (char *) BLU_buf, i = 0; i < 6; ++i, ++s)
                  fprintf(trace, "%02X ", (int) *s & 0xFF);
               fprintf(trace, "\n");
               break;
         }  // End of BLU_buf[Fptr+FCntl] & 0x03
         break;

      case SUPRV:
         // *** SUPERVISORY FORMAT ***
         switch (BLU_buf[Fptr + FCntl] & 0x0F) {
            case RR:
               fprintf(trace, "RR - N(r)=%d, PF=%d \n", Nr, PF);
               break;

            case RNR:
               fprintf(trace, "RNR - N(r)=%d, PF=%d \n", Nr, PF);
               break;

         }  // End of switch (BLU_buf[Fptr+FCntl] & 0x0F)

      case IFRAME:
      case IFRAME + 0x02:              // ..00 & ..10 are I-frames
         // *** INFORMATIONAL FRAME ***
         fprintf(trace, "IFRAME - N(r)=%d, PF=%d, N(s)=%d - BLU_buf[%d]=",
            Nr, PF, Ns, Fptr);

         for (s = (char *) BLU_buf, i = 0; i < Flen; ++i, ++s)
            fprintf(trace, "%02X ", (int) *s & 0xFF);
         fprintf(trace, "\n");
         break;

   }  // End of switch (BLU_buf[Fptr+FCntl] & 0x03)
}

