/* i3705_i2c_defs.h: IBM 3705 CCU panel simulator definitions

   Copyright (c) 2020, Henk Stegeman

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

   Except as contained in this notice, the name of Robert M Supnik shall not
   be used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

*/

/* MCP23017 register definitions */

#define IODIRA            0x00      /* I/O Direction */
#define IODIRB            0x01
#define IPOLA             0x02      /* Polarity */
#define IPOLB             0x03
#define GPINTENA          0x04      /* Interrupt on change */
#define GPINTENB          0x05
#define DEFVALA           0x06      /* Default value */
#define DEFVALB           0x07
#define INTCONA           0x08      /* Interrupt control */
#define INTCONB           0x09
#define IOCON             0x0A      /* I/O Configuration */
#define IOCONB            0x0B
#define GPPPUA            0x0C      /* GPIO pull-up resistor */
#define GPPPUB            0x0D
#define INTFA             0x0E      /* Interrupt flag */
#define INTFB             0x0F
#define INTCAPA           0x10      /* Interrupt capture */
#define INTCAPB           0x11
#define GPIOA             0x12      /* Port register */
#define GPIOB             0x13
#define OLATA             0x14      /* Output Latch */
#define OLATB             0x15

/* General equates */

#define RELEASED          0
#define PRESSED           1
#define ACTED             2

/* For CPU --- Panel comm. */
/* cpu_cmd */

#define NORMAL            0x00
#define BOOT_CPU          0x02
#define CPU_RESET         0x03
#define DISP_STOR         0x04
#define ALTER_STOR        0x05
#define TERMINATE         0x09

/* cpu_state */

#define STOP              0x00
#define STOPPED           0x01
#define RUN               0x02
#define RUNNING           0x03

/* stop_rc */

#define FROM_SIMH         0x10
#define RETURN_SIMH       0x11

/* Operator panel switches */
/* MoDe selector switch */

#define MD_ADDR_CMP_INT   0x01
#define MD_PROCESS        0x02
#define MD_ADDR_CMP_STOP  0x04
#define MD_INSTR_STEP     0x08

/* Display & Function selector */

#define DF_TAR_OP_REG   0x1000
#define DF_STATUS       0x2000
#define DF_STOR_ADDR    0x4000
#define DF_REG_ADDR     0x8000
#define DF_FUNC_1       0x0002
#define DF_FUNC_2       0x0004
#define DF_FUNC_3       0x0008
#define DF_FUNC_4       0x0010
#define DF_FUNC_5       0x0020
#define DF_FUNC_6       0x0040

/* Buttons */

#define BT_SET_A_D        0x01
#define BT_START          0x02
#define BT_STOP           0x04
#define BT_INTERRUPT      0x08
#define BT_LOAD           0x10
#define BT_RESET          0x20

/* Operator panel lights */
/* Status lights */

#define LT_HARD_STOP      0x01
#define LT_TEST           0x02
#define LT_WAIT           0x04
#define LT_PGM_STOP       0x08
#define LT_LOAD           0x10

/* Display A lights */

#define LT_DISP_A         0x00

/* Display B lights */

#define LT_DISP_B         0x00

