/* COMM3705.H   (c) Copyright Max H. Parke, 2007-2012                */
/*              Hercules 3705 communications controller              */
/*              running NCP                                          */

// #include "hercules.h"

#define TIDPAT             "%8.8"PRIx32       // complete format spec
#define SCN_TIDPAT            "%"PRIx32       // complete format spec

struct COMMADPT
{
   DEVBLK *dev;                 /* The devblk to which this CA is attched   */
   TID  cthread;                /* Thread used to control the socket        */
   TID  tthread;                /* Thread used to control the socket        */
   U16  lport;                  /* Local listening port                     */
   in_addr_t lhost;             /* Local listening address                  */
   int  sfd;                    /* Communication socket FD                  */
   int  lfd;                    /* Listen socket for DIAL=IN, INOUT & NO    */
   COND ipc;                    /* I/O <-> thread IPC condition EVB         */
   COND ipc_halt;               /* I/O <-> thread IPC HALT special EVB      */
   LOCK lock;                   /* COMMADPT lock                            */
   int  pipe[2];                /* pipe used for I/O to thread signaling    */
   char locncpnm[9],            /* name of local NCP (in EBCDIC)            */
        rmtncpnm[9];            /* name of remote NCP (in EBCDIC)           */
   U16  devnum,                 /* devnum copy from DEVBLK                  */
        locsuba,                /* Local NCP or local 3791 node subarea nr  */
        rmtsuba;                /* Remote NCP subarea number                */
   U32
        have_cthread:1,         /* the comm thread is running               */
        haltpending:1,          /* req has been issued to halt current CCW  */
        bindflag:1,
        initselfflag:1,
        telnet_opt:1,           /* expecting telnet option char             */
        telnet_iac:1,           /* expecting telnet command char            */
        telnet_int:1,           /* telnet interrupt received                */
        hangup:1,               /* host initated shutdown                   */
        is_3270:1,              /* 0 = tty, 1 = 3270                        */
        eol_flag:1,             /* 1 = CR has been received                 */
        debug_sna:1,            /* 1 = write debug messages                 */
        emu3791:1,              /* mode (0=default=3705;1=3791)             */
        idblk,                  /* IDBLK of switched PU (default=0x017)     */
        idnum;                  /* IDNUM of switched PU (default=0x00017)   */
    U32 rlen3270;               /* amt of data in 3270 recv buf             */
    BYTE telnet_cmd;            /* telnet command                           */

    int ncpa_sscp_seqn;
    int ncpb_sscp_seqn;
    int lu_sscp_seqn;
    int lu_lu_seqn;

    BYTE inpbuf[65536];

    int  inpbufl,
         unitsz,                /* I/O blocksize (default = 256)            */
         ackspeed;              /* slow down factor for unacknowledged attn */

    void *freeq;
    void *sendq;
    BYTE *poolarea;

    BYTE sscp_addr0;
    BYTE sscp_addr1;
    BYTE ncp_addr0;
    BYTE ncp_addr1;
    BYTE pu_addr0;
    BYTE pu_addr1;
    BYTE lu_addr0;
    BYTE lu_addr1;
    BYTE tso_addr0;
    BYTE tso_addr1;
};

#define BUFLEN_3270     65536           /* 3270 Send/Receive buffer  */
#define BUFLEN_1052     150             /* 1052 Send/Receive buffer  */

/*-------------------------------------------------------------------*/
/* Telnet command definitions                                        */
/*-------------------------------------------------------------------*/
#define BINARY          0       /* Binary Transmission */
#define IS              0       /* Used by terminal-type negotiation */
#define SEND            1       /* Used by terminal-type negotiation */
#define ECHO_OPTION     1       /* Echo option */
#define SUPPRESS_GA     3       /* Suppress go-ahead option */
#define TIMING_MARK     6       /* Timing mark option */
#define TERMINAL_TYPE   24      /* Terminal type option */
#define NAWS            31      /* Negotiate About Window Size */
#define EOR             25      /* End of record option */
#define EOR_MARK        239     /* End of record marker */
#define SE              240     /* End of subnegotiation parameters */
#define NOP             241     /* No operation */
#define DATA_MARK       242     /* The data stream portion of a Synch.
                                   This should always be accompanied
                                   by a TCP Urgent notification */
#define BRK             243     /* Break character */
#define IP              244     /* Interrupt Process */
#define AO              245     /* Abort Output */
#define AYT             246     /* Are You There */
#define EC              247     /* Erase character */
#define EL              248     /* Erase Line */
#define GA              249     /* Go ahead */
#define SB              250     /* Subnegotiation of indicated option */
#define WILL            251     /* Indicates the desire to begin
                                   performing, or confirmation that
                                   you are now performing, the
                                   indicated option */
#define WONT            252     /* Indicates the refusal to perform,
                                   or continue performing, the
                                   indicated option */
#define DO              253     /* Indicates the request that the
                                   other party perform, or
                                   confirmation that you are expecting
                                   the other party to perform, the
                                   indicated option */
#define DONT            254     /* Indicates the demand that the
                                   other party stop performing,
                                   or confirmation that you are no
                                   longer expecting the other party
                                   to perform, the indicated option  */
#define IAC             255     /* Interpret as Command              */

