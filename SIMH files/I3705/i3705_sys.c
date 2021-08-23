/* s3_sys.c: IBM 3705 system interface

   Copyright (c) 2020, Henk Stegeman & Edwin Freekenhorst

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
   
*/

#include <ctype.h>
#include "i3705_defs.h"

extern DEVICE cpu_dev;
extern UNIT cpu_unit;
extern REG cpu_reg[];
extern FILE *trace;        // DEBUG HJS

extern int32 lvl;
extern int32 Grp;
extern int32 GR[8][4];
extern int8  CL_C[4], CL_Z[4];
extern int8  test_mode;
extern int32 Eregs_Inp[128];
extern int32 Eregs_Out[128];
extern unsigned char M[];
extern int32 saved_PC;   
//extern unsigned char ebcdic_to_ascii[];
char *parse_addr(char *cptr,  char *gbuf, t_addr *addr, int32 *addrtype);

int32 printf_sym (FILE *of, char *strg, t_addr addr, uint32 *val,
    UNIT *uptr, int32 sw);

int32 R1fld, R2fld, Rfld;
int32 N1fld, N2fld, Nfld;
int32 Afld, Bfld, Dfld, Efld, Ifld, Mfld, Tfld;
int32 Rgrp;
char buff[80];

/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
   sim_load             binary loader
*/

char  sim_name[] = "IBM 3704/3705";
REG  *sim_PC = &cpu_reg[0];     
int32 sim_emax = 4;
DEVICE *sim_devices[] = { 	
     &cpu_dev, 
     NULL };
const char *sim_stop_messages[] = {
    "Unknown error",
    "Unknown I/O Instruction",
    "HALT instruction",
    "Entered BP after execution",
    "Invalid Opcode",
    "Invalid Qbyte",
    "Invalid Address",
    "Invalid Device Command",
    "ATTN Card Reader"
};

/* This is the opcode master defintion table.  Each possible instr mnemonic
   is defined here, with enough information to translate to and from
   symbolic to binary machine code.  
   First field is the instruction's mnemonic
*/   

int32 nopcode = 55;

struct opdef optable[55] = {
//    Mnem   opcode  opmask frm grp  
    {"B  " , 0xA800, 0xF800, 3, 0},
    {"BCL" , 0x9800, 0xF800, 3, 0},
    {"BZL" , 0x8800, 0xF800, 3, 0},
    {"BCT" , 0xB880, 0xF880,10, 0},
    {"BB " , 0xC800, 0xF800, 6, 0},
    {"BB " , 0xD800, 0xF800, 6, 0},
    {"BB " , 0xE800, 0xF800, 6, 0},
    {"BB " , 0xF800, 0xF800, 6, 0},

    {"LRI" , 0x8000, 0xF800, 2, 0},
    {"ARI" , 0x9000, 0xF800, 2, 0},
    {"SRI" , 0xA000, 0xF800, 2, 0},
    {"CRI" , 0xB000, 0xF800, 2, 0},
    {"XRI" , 0xC000, 0xF800, 2, 0},
    {"ORI" , 0xD000, 0xF800, 2, 0},
    {"NRI" , 0xE000, 0xF800, 2, 0},
    {"TRM" , 0xF000, 0xF800, 2, 0},

    {"LCR" , 0x0008, 0x88FF, 1, 0},
    {"ACR" , 0x0018, 0x88FF, 1, 0},
    {"SCR" , 0x0028, 0x88FF, 1, 0},
    {"CCR" , 0x0038, 0x88FF, 1, 0},
    {"XCR" , 0x0048, 0x88FF, 1, 0},
    {"OCR" , 0x0058, 0x88FF, 1, 0},
    {"NCR" , 0x0068, 0x88FF, 1, 0},
    {"LCOR", 0x0078, 0x88FF, 1, 0},

    {"ICT" , 0x0010, 0x88FF, 5, 0},
    {"STCT", 0x0030, 0x88FF, 5, 0},
    {"IC " , 0x0800, 0x8880, 5, 1},
    {"STC" , 0x0880, 0x8880, 5, 1},

    {"LH " , 0x0001, 0x8881, 7, 0},
    {"STH" , 0x0081, 0x8881, 7, 0},
    {"L  " , 0x0002, 0x8883, 7, 1},
    {"ST " , 0x0082, 0x8883, 7, 1},

    {"LHR" , 0x0080, 0x88FF, 0, 0},
    {"AHR" , 0x0090, 0x88FF, 0, 0},
    {"SHR" , 0x00A0, 0x88FF, 0, 0},
    {"CHR" , 0x00B0, 0x88FF, 0, 0},
    {"XHR" , 0x00C0, 0x88FF, 0, 0},
    {"OHR" , 0x00D0, 0x88FF, 0, 0},
    {"NHR" , 0x00E0, 0x88FF, 0, 0},
    {"LHOR", 0x00F0, 0x88FF, 0, 0},
    {"LR " , 0x0088, 0x88FF, 0, 0},
    {"AR " , 0x0098, 0x88FF, 0, 0},
    {"SR " , 0x00A8, 0x88FF, 0, 0},
    {"CR " , 0x00B8, 0x88FF, 0, 0},
    {"XR " , 0x00C8, 0x88FF, 0, 0},
    {"OR " , 0x00D8, 0x88FF, 0, 0},
    {"NR " , 0x00E8, 0x88FF, 0, 0},
    {"LOR" , 0x00F8, 0x88FF, 0, 0},
    {"BALR", 0x0040, 0x88FF, 0, 0},

    {"IN " , 0x000C, 0x880F, 8, 1},
    {"OUT" , 0x0004, 0x880F, 8, 0},

    {"BAL" , 0xB800, 0xF8F0, 4, 0},
    {"LA " , 0xB820, 0xF8F0, 4, 0},

    {"EXIT", 0xB840, 0xFFFF, 9, 0},

    {"INV",  0x0000, 0xFFFF,11, 0}   
};

/* This is the binary loader.  The input file is considered to be
   a string of literal bytes with no special format. The
   load starts at the addr specified in the TXT record.
*/

t_stat sim_load (FILE *fileref, char *cptr, char *fnam, int flag) {
   int32 i, j, addr = 0x0000, cnt = 0;

   if ((*cptr != 0) || (flag != 0)) return SCPE_ARG;
   addr = 0x0000;
   i = 0;

   while (fread(&buff, 1, 80, fileref) == 80) {
      if (buff[2] == 0xE7) {                          /* TXT record ? */
         cnt = buff[11];                              /* Get byte count */
         addr = (buff[6] << 8) | buff[7];             /* Get address to store */
         for (j = cnt; j > 0; j--) {
            M[addr] = buff[16 + (cnt - j) ];          /* Store it */
            addr++;
            i++;                                      /* Bump byte count */
         }
      } else {
         continue;                                    /* Next record please */
      }
   }
   printf("\n\r");
   printf ("%d Bytes loaded. Last byte stored at loc %05X.\n", i, addr-1);
   return (SCPE_OK);
}

/* Symbolic output

   Inputs:
        *of     =  output stream
        addr    =  current PC
        *val    =  pointer to values
        *uptr   =  pointer to unit
        sw      =  switches
   Outputs:
        status  =  error code
*/

t_stat fprint_sym (FILE *of, t_addr addr, uint32 *val,
   UNIT *uptr, int32 sw) 
   {
   int32 r;
   char strg[256];
   
   strcpy(strg, "");
   r = printf_sym(of, strg, addr, val, uptr, sw);
   if (sw & SWMASK ('A'))
      strcpy(strg, "");
   else
      fprintf(of, "%s", strg);
   return (r);
}


t_stat printf_sym (FILE *of, char *strg, t_addr addr, uint32 *val,
    UNIT *uptr, int32 sw)
{
int32 c1, c2, group, inst;
int32 oplen, i, j;
char  bld[128], bldaddr[36], bldregs[80];
int32 blk[16], blt[16];
int32 blkadd;

c1 = val[0] & 0xff;
if (sw & SWMASK ('A')) {
   for (i = 0; i < 16; i++) {
      blkadd = addr + (i*16);
      for (j = 0; j < 16; j++) {
         blk[j] = M[blkadd+j] & 0xff;
//          c2 = ebcdic_to_ascii[blk[j]];
         if (c2 < 040 || c2 > 0177 || blk[j] == 07) {
            blt[j] = '.';
         } else {    
            blt[j] = c2;
         }
      }
      if (i == 0) {
         fprintf(of, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X  [%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c]\n ",
                    blk[0], blk[1], blk[2], blk[3], blk[4], blk[5], blk[6], blk[7],
                    blk[8], blk[9], blk[10], blk[11], blk[12], blk[13], blk[14], blk[15],
                    blt[0], blt[1], blt[2], blt[3], blt[4], blt[5], blt[6], blt[7],
                    blt[8], blt[9], blt[10], blt[11], blt[12], blt[13], blt[14], blt[15]);
      } else {
         fprintf(of, "%X\t%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X  [%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c]\n ",
                    blkadd, blk[0], blk[1], blk[2], blk[3], blk[4], blk[5], blk[6], blk[7],
                    blk[8], blk[9], blk[10], blk[11], blk[12], blk[13], blk[14], blk[15],
                    blt[0], blt[1], blt[2], blt[3], blt[4], blt[5], blt[6], blt[7],
                    blt[8], blt[9], blt[10], blt[11], blt[12], blt[13], blt[14], blt[15]);
      }           
   }       
   return SCPE_OK; 
}

if (sw & SWMASK ('C')) {
//    c2 = ebcdic_to_ascii[c1];
   if (c2 < 040 || c2 > 0177) {
      sprintf(strg, "<%04X>", c1 & 0xffff);
   } else {    
      sprintf(strg, "%c", c2 & 0xff);
   }
   return SCPE_OK; 
}

if (!(sw & SWMASK ('M'))) return SCPE_ARG;

inst = ((val[0] << 8) | val[1]);

/* Find the table entry */

for (i = 0; i < nopcode; i++) {
   if ((inst & optable[i].opmask) == optable[i].opcode) 
      break;
}

/* Print the mnem opcode */

if (i >= nopcode) 
   sprintf(bld, "'%04X' Invalid instruction! ", inst);
else { 
   sprintf(bld, " %s", optable[i].mnem);

/* Display the operands in the correct format */
switch (optable[i].form) {
   case 0:  // RR format: R1,R2
      R1fld = (val[0] & 0x07);
      R2fld = (val[0] >> 4) & 0x07;
      sprintf(bldaddr, " R%01X,R%01X ", R1fld, R2fld);
      break;
   case 1:  // RRn format: R1(N1),R2(N2)
      R1fld = (val[0] & 0x06) + 1;
      if (val[0] & 0x01)
         N1fld = 'L';
      else
         N1fld = 'H';
      R2fld = ((val[0] >> 4) & 0x06) + 1;
      if ((val[0] >> 4) & 0x01)
         N2fld = 'L';
      else
         N2fld = 'H';
      sprintf(bldaddr, " R%01X(%1c),R%01X(%1c) ", R1fld, N1fld, R2fld, N2fld);
      break;
   case 2:  // RI format: R(N),I
      Rfld = ((val[0] & 0x06) + 1);
      if (val[0] & 0x01) 
         Nfld = 'L';
      else
         Nfld = 'H';
      Ifld = (val[1]);
      sprintf(bldaddr, " R%01X(%1c),I=%02X ", Rfld, Nfld, Ifld);
      break;
   case 3:  // RT format: T
      Tfld = inst & 0x07FE;
      if (inst & 0x0001)     // + or - ?
         sprintf(bldaddr, " T=-%03X ", Tfld );
      else
         sprintf(bldaddr, " T=+%03X ", Tfld );
      break;
   case 4:  // RA format: R,A
      Rfld = (val[0] & 0x07);
      Afld = (val[1] & 0x0F) << 16;
      Afld = Afld | (val[2] << 8) | val[3];  
      sprintf(bldaddr, " R%01X,A=%05X ", Rfld, Afld); 
      break;
   case 5:  // RS format: R(N),D(B)
      Rfld = (val[0] & 0x06) + 1;
      Bfld = (val[0] >> 4) & 0x07;
      Dfld = (val[1]) & 0x7F;
      if (val[0] & 0x01)
         Nfld = 'L';
      else
         Nfld = 'H';
      if (optable[i].group == 0) {     // No displacement instr.
         sprintf(bldaddr, " R%01X(%1c),B=R%01X  [0x%04X] ", Rfld, Nfld, Bfld, GR[Bfld][Grp]);
      } else {
         if (Bfld > 0)  
            sprintf(bldaddr, " R%01X(%1c),D=%02d(B=R%01X)  [0x%02X]+[0x%04X] ", 
                    Rfld, Nfld, Dfld, Bfld, Dfld, GR[Bfld][Grp] );
         else
            sprintf(bldaddr, " R%01X(%1c),D=%02d(B=R%01X)  [0x%02X]+[0x%04X] ", 
                    Rfld, Nfld, Dfld, Bfld, Dfld, 0x0680 );
      }  
      break;
   case 6:  // BTm format: R(N,M),T
      Rfld = ((val[0] & 0x06) + 1);
      if (val[0] & 0x01)
         Nfld = 'L';
      else
         Nfld = 'H';
      Mfld = ((val[0] >> 3) & 0x06) | ((val[1] >> 7) & 0x01); 
      Tfld = inst & 0x007E;
      if (inst & 0x0001)               // + or - ?
         sprintf(bldaddr, " R%01X(%1c,M=%01X),T=-%02X ", Rfld, Nfld, Mfld, Tfld);
      else
         sprintf(bldaddr, " R%01X(%1c,M=%01X),T=+%02X ", Rfld, Nfld, Mfld, Tfld);
      break; 
   case 7:  // RS format: R,D(B)
      Rfld = (val[0]) & 0x07;
      Bfld = (val[0] >> 4) & 0x07;
      Dfld = val[1] & 0x7E;
      if (optable[i].group == 0) {     // LH / STH with 6 bits displacement
         if (Bfld > 0)
            sprintf(bldaddr, " R%01X,(D=%02d)B=R%01X  [0x%02X]+[0x%04X] ", 
                    Rfld, Dfld, Bfld, Dfld & 0x7E, GR[Bfld][Grp]);
         else
            sprintf(bldaddr, " R%01X,(D=%02d)B=R%01X  [0x%02X]+[0x%04X] ", 
                    Rfld, Dfld, Bfld, Dfld & 0x7E, 0x0700);
      } else {                         // L / ST with 5 bits displacement 
         if (Bfld > 0)
            sprintf(bldaddr, " R%01X,(D=%02d)B=R%01X  [0x%02X]+[0x%05X] ", 
                    Rfld, Dfld, Bfld, Dfld & 0x7C, GR[Bfld][Grp]);
         else 
            sprintf(bldaddr, " R%01X,(D=%02d)B=R%01X  [0x%02X]+[0x%05X] ", 
                    Rfld, Dfld, Bfld, Dfld & 0x7C, 0x0780);
      }  
      break;
   case 8:  // RE format: R,E 
      Efld = (val[0] & 0x70) | (val[1] >> 4);
      Rfld = (val[0] & 0x07);
      if (Efld < 0x20) 
         sprintf(bldaddr, " R%01X,E=%02X --- [0x%04X]", Rfld, Efld, GR[Efld & 0x07][Efld >> 3]); 
      else {   // 0x20 - 0x7F
         if (optable[i].group == 1) {        // Input instruction ?  
            sprintf(bldaddr, " R%01X,E=%02X <-  [0x%04X] ", Rfld, Efld, Eregs_Inp[Efld]);
         } else {                            // Output instruction ? 
            sprintf(bldaddr, " R%01X,E=%02X  -> [0x%04X] ", Rfld, Efld, GR[Rfld][Grp]);
            if (Efld == 0x45)    // DEBUG HJS 
               fprintf(trace, ">>> OUT  R%01X,E=%02X  -> [0x%04X] ", Rfld, Efld, GR[Rfld][Grp]);
         }
      }
      break;
   case 9:  // EXIT
      sprintf(bldaddr, " Leaving lvl %d", lvl );
      break;
   case 10: // RT format: R(N),T
      Tfld = inst & 0x007E;
      Rfld = ((val[0] & 0x06) + 1);
      if (val[0] & 0x01)
         Nfld = 'L';
      else
         Nfld = 'H';
      if (inst & 0x0001)     // + or - ?
         sprintf(bldaddr, " R%01X(%c),T=-%02X ", Rfld, Nfld, Tfld );
      else
         sprintf(bldaddr, " R%01X(%c) T=+%02X ", Rfld, Nfld, Tfld );
      break;
   case 11:  // INVALID INSTRUCTION
         sprintf(bldaddr, " INSTRUCTION " );
      break;
}     

sprintf(bldregs, "Lvl=%d Grp=%d | %05X %05X %05X %05X  %05X %05X %05X %05X | C=%d Z=%d T=%d",
        lvl, Grp,
        GR[0][Grp], GR[1][Grp], GR[2][Grp], GR[3][Grp],
        GR[4][Grp], GR[5][Grp], GR[6][Grp], GR[7][Grp],
        CL_C[Grp], CL_Z[Grp], test_mode);

sprintf(strg, "%s%s\n%s", bld, bldaddr, bldregs);
}
oplen = 22;

return -(oplen - 1);
}

t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
int32 cflag, i = 0, j, r, oplen, addtyp, saveaddr, vptr;
char gbuf[CBUFSIZE];

cflag = (uptr == NULL) || (uptr == &cpu_unit);
while (isspace (*cptr)) cptr++;                         /* absorb spaces */
if ((sw & SWMASK ('A')) || ((*cptr == '\'') && cptr++)) { /* ASCII char? */
    if (cptr[0] == 0) return SCPE_ARG;                  /* must have 1 char */
    val[0] = (unsigned int) cptr[0];
    return SCPE_OK;
}
if ((sw & SWMASK ('C')) || ((*cptr == '"' ) && cptr++)) { /* ASCII string? */
    if (cptr[0] == 0) return SCPE_ARG;                  /* must have 1 char */
    val[0] = ((unsigned int) cptr[0] << 8) + (unsigned int) cptr[1];
    return SCPE_OK;
}
}
