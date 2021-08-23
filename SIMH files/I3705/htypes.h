/* HTYPES.H     (c) Copyright Roger Bowler, 1999-2012                */
/*              Hercules Type Definitions                            */
/*                                                                   */
/*   Released under "The Q Public License Version 1"                 */
/*   (http://www.hercules-390.org/herclic.html) as modifications to  */
/*   Hercules.                                                       */

#define HAVE_INTTYPES_H
#define HAVE_U_INT8_T

/*-------------------------------------------------------------------*/
/*                      Fish Threads                                 */
/*-------------------------------------------------------------------*/
typedef pthread_t               hthread_t;
typedef hthread_t               HID;    /* Hercules thread-id type   */
typedef HID                     TID;    /* Generic thread-id type    */
typedef pthread_cond_t          COND;
typedef pthread_attr_t          ATTR;
typedef pthread_mutexattr_t     MATTR;
typedef pthread_mutex_t         LOCK;

typedef struct _parser
{
   char *key;
   char *fmt;
} PARSER;

//efine  uint8_t    u_int8_t
//efine  uint16_t   u_int16_t
//efine  uint32_t   u_int32_t
//efine  uint64_t   u_int64_t

//pedef  uint8_t    u_int8_t;
//pedef  uint16_t   u_int16_t;
//pedef  uint32_t   u_int32_t;
//pedef  uint64_t   u_int64_t;

#define  SIZEOF_INT_P     4
#define  SIZEOF_SIZE_T    4

typedef  int8_t     S8;         // signed 8-bits
typedef  int16_t    S16;        // signed 16-bits
typedef  int32_t    S32;        // signed 32-bits
typedef  int64_t    S64;        // signed 64-bits

typedef  uint8_t    U8;         // unsigned 8-bits
typedef  uint16_t   U16;        // unsigned 16-bits
typedef  uint32_t   U32;        // unsigned 32-bits
typedef  uint64_t   U64;        // unsigned 64-bits

typedef  uint8_t    BYTE;       // unsigned byte       (1 byte)
typedef  uint8_t    HWORD[2];   // unsigned halfword   (2 bytes)
typedef  uint8_t    FWORD[4];   // unsigned fullword   (4 bytes)
typedef  uint8_t    DBLWRD[8];  // unsigned doubleword (8 bytes)
typedef  uint8_t    QWORD[16];  // unsigned quadword   (16 bytes)

/*-------------------------------------------------------------------*/
/* Socket stuff                                                      */
/*-------------------------------------------------------------------*/
typedef unsigned char   u_char;
typedef unsigned short  u_short;
typedef unsigned int    u_int;
typedef unsigned long   u_long;

typedef unsigned int    socklen_t;

typedef unsigned int    in_addr_t;



  // (The following are simply to silence some compile time warnings)
#ifdef _MSVC_
  typedef  char               GETSET_SOCKOPT_T;
  typedef  const char *const *EXECV_ARG2_ARGV_T;
#else
  typedef  void         GETSET_SOCKOPT_T;
  typedef  char *const *EXECV_ARG2_ARGV_T;
#endif

/*-------------------------------------------------------------------*/
/* Primary Hercules Control Structures                               */
/*-------------------------------------------------------------------*/
typedef struct SYSBLK    SYSBLK;    // System configuration block
typedef struct REGS      REGS;      // CPU register context
typedef struct VFREGS    VFREGS;    // Vector Facility Registers
typedef struct ZPBLK     ZPBLK;     // Zone Parameter Block
typedef struct TELNET    TELNET;    // Telnet Control Block
typedef struct DEVBLK    DEVBLK;    // Device configuration block
typedef struct CHPBLK    CHPBLK;    // Channel Path config block
typedef struct IOINT     IOINT;     // I/O interrupt queue

typedef struct GSYSINFO  GSYSINFO;  // Ebcdic machine information

typedef struct DEVDATA   DEVDATA;   // xxxxxxxxx
typedef struct DEVGRP    DEVGRP;    // xxxxxxxxx
typedef struct DEVHND    DEVHND;    // xxxxxxxxx
typedef struct SHRD      SHRD;      // xxxxxxxxx

/*-------------------------------------------------------------------*/
/* Secondary Device and I/O Control Related Structures               */
/*-------------------------------------------------------------------*/

typedef struct CKDDASD_DEVHDR   CKDDASD_DEVHDR;   // Device header
typedef struct CKDDASD_TRKHDR   CKDDASD_TRKHDR;   // Track header
typedef struct CKDDASD_RECHDR   CKDDASD_RECHDR;   // Record header
typedef struct CCKDDASD_DEVHDR  CCKDDASD_DEVHDR;  // Compress device header
typedef struct CCKD_L2ENT       CCKD_L2ENT;       // Level 2 table entry

typedef struct CCKD_FREEBLK     CCKD_FREEBLK;     // Free block
typedef struct CCKD_IFREEBLK    CCKD_IFREEBLK;    // Free block (internal)
typedef struct CCKD_RA          CCKD_RA;          // Readahead queue entry

typedef struct CCKDBLK          CCKDBLK;          // Global cckd dasd block
typedef struct CCKDDASD_EXT     CCKDDASD_EXT;     // Ext for compressed ckd

typedef struct COMMADPT         COMMADPT;         // Comm Adapter
typedef struct bind_struct      bind_struct;      // Socket Device Ctl

typedef struct TAPEMEDIA_HANDLER  TAPEMEDIA_HANDLER;  // (see tapedev.h)
typedef struct TAPEAUTOLOADENTRY  TAPEAUTOLOADENTRY;  // (see tapedev.h)
typedef struct TAMDIR             TAMDIR;             // (see tapedev.h)

/*-------------------------------------------------------------------*/
/* Device handler function prototypes                                */
/*-------------------------------------------------------------------*/

typedef int   DEVIF  (DEVBLK *dev, int argc, char *argv[]);
typedef void  DEVQF  (DEVBLK *dev, char **devclass, int buflen,
                                   char *buffer);
typedef void  DEVXF  (DEVBLK *dev, BYTE code, BYTE flags,
                                   BYTE chained, U32 count,
                                   BYTE prevcode, int ccwseq,
                                   BYTE *iobuf, BYTE *more,
                                   BYTE *unitstat, U32 *residual);
typedef void  DEVHF  (DEVBLK *dev);
typedef int   DEVCF  (DEVBLK *dev);
typedef void  DEVSF  (DEVBLK *dev);
typedef int   DEVRF  (DEVBLK *dev, int ix, BYTE *unitstat);
typedef int   DEVWF  (DEVBLK *dev, int rcd, int off, BYTE *buf,
                                   int len, BYTE *unitstat);
typedef int   DEVUF  (DEVBLK *dev);
typedef void  DEVRR  (DEVBLK *dev);
typedef int   DEVSA  (DEVBLK *dev, U32 qmask);
typedef int   DEVSAS (DEVBLK *dev, U32 oqmask, U32 iqmask);
typedef int   DEVQD  (DEVBLK *dev, void *desc);
typedef int   DEVSR  (DEVBLK *dev, void *file);

/*-------------------------------------------------------------------*/
/* Device handler description structures                             */
/*-------------------------------------------------------------------*/
typedef BYTE *DEVIM;                    /* Immediate CCW Codes Table */

/*-------------------------------------------------------------------*/
/* Read Configuration Data function                                  */
/*-------------------------------------------------------------------*/
typedef int   DEVRCD  (DEVBLK *dev, BYTE *buffer, int bufsz);

