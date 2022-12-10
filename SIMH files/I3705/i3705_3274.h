/* i3705_3274.h     (c) Copyright Edwin Freekenhorst & Henk Stegeman */
/*              based on comm3705.h from Max H. Parke                */
/*                                                                   */


// #include "hercules.h"

#define TIDPAT             "%8.8"PRIx32       // complete format spec
#define SCN_TIDPAT            "%"PRIx32       // complete format spec


#define BUFLEN_3270     65536           /* 3270 Send/Receive buffer  */
#define BUFLEN_1052     150             /* 1052 Send/Receive buffer  */
#define MAXPU 2                    // Max number of 3274
#define MAXLU 4                    // Max number of terminals per PU that can connect

/*-------------------------------------------------------------------*/
/*3270 Data Structure                                                */
/*-------------------------------------------------------------------*/
struct IO3270 {
   uint8_t  inpbuf[65536];
   uint32_t inpbufl;
} *ioblk[MAXPU][MAXLU];

/*-------------------------------------------------------------------*/
/*3274 Data Structure                                                */
/*-------------------------------------------------------------------*/
struct CBPU2 {
   int      lu_fd[MAXLU];
   int      punum;
   int      lunum;
   int      pu_fd;
   int      epoll_fd;
   uint32_t actlu[MAXLU];
   uint32_t is_3270[MAXLU];
   uint32_t rlen3270[MAXLU];           /* size of data in 3270 receive buffer   */
   uint32_t bindflag[MAXLU];
   uint32_t initselfflag[MAXLU];
   uint32_t telnet_opt[MAXLU];         /* expecting telnet option char          */
   uint32_t telnet_iac[MAXLU];         /* expecting telnet command char         */
   uint32_t telnet_int[MAXLU];         /* telnet intterupt received             */
   uint32_t eol_flag[MAXLU];           /* Carriage Return received              */
   int      ncpa_sscp_seqn;
   //int      lu_sscp_seqn;
   int      lu_lu_seqn[MAXLU];
   uint8_t  telnet_cmd[MAXLU];         /* telnet command                        */
   uint8_t  last_P_Ns;
   uint8_t  last_S_Ns;
   uint8_t  new_S_Ns;
   uint8_t  new_S_Nr;
   uint8_t  sscp_addr0;
   uint8_t  sscp_addr1;
   uint8_t  pu_addr0;
   uint8_t  pu_addr1;
   uint8_t  lu_addr0;
   uint8_t  lu_addr1;
   uint8_t  daf_addr1[MAXLU];
   uint8_t  last_lu;
}  *pu2[MAXPU];

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

