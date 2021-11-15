/******************************************************************/
/* COMM3705.C	(c) Copyright Edwin Freekenhorst and Henk Stegeman    */
/*                                                                    															*/ 
/* Hercules channel extender for the 								  									*/
/* 			3705 communications controller simulator                  						*/
/*                                                                   																*/   
/* This version uses a skeleton version of the COMM3705.C            				*/
/* coding from Max H. Parke                                           										*/          
/*                                                                    															*/
/* The purpose of this module is to extend the 3705 Channel Adapter   		*/
/* module of the 3705 simulator, into hercules.                    	  							*/
/* The core function is to maintain a dual TCP/IP connection with the 			*/
/* 3705 simulator, emulating a bus and tag channel connection.	      			*/
/*																	  															*/
/******************************************************************/


#include "hstdinc.h"
#include "hercules.h"
#include "devtype.h"
#include "opcode.h"
#include "parser.h"
#include "stdbool.h"



#if defined(WIN32) && defined(OPTION_DYNAMIC_LOAD) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
  SYSBLK *psysblk;
  #define sysblk (*psysblk)
#endif

#if !defined(min)
#define  min(a,b)              (((a) <= (b)) ? (a) : (b))
#endif



#define BUFPD 0x1C

static BYTE commadpt_immed_command[256]=
{ 0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*---------------------------------------------------------------*/
/* PARSER TABLES                                                 */
/*---------------------------------------------------------------*/

static PARSER ptab[] = {
    {"port",   "%s"},
    {"adaptip","%s"},
    {"debug",   "%s"},
    {NULL, NULL}
};


enum {
    COMMADPT_KW_PORT = 1,
    COMMADPT_KW_ADAPTIP,
    COMMADPT_KW_DEBUG,
} comm3705_kw;



struct COMMADPT
   {
   DEVBLK *dev;            /* the devblk to which this CA is attched */
   TID  cthread;           /* Thread used to control the socket      */
   TID  tthread;           /* Thread used to control the socket      */
   U16  port;            /* Local listening port                   */
   BYTE *unitstat;         /* pointer to channel unitstat            */
   in_addr_t adaptip;       /* Local listening address                */
   BYTE carnstat;          /* Channel Adaptor return status          */
   BYTE ccwactive;         /* indicate an active CCW                 */
   int busfd;              /* Communication socket for ccw/data      */
   int tagfd;              /* Communication socket for attn          */
   COND ipc;               /* I/O <-> thread IPC condition EVB       */
   COND ipc_halt;          /* I/O <-> thread IPC HALT special EVB    */
   LOCK lock;              /* COMMADPT lock                          */
   int pipe[2];            /* pipe used for I/O to thread signaling  */
   U16  devnum;            /* devnum copy from DEVBLK                */
   struct sockaddr_in servaddr;
 
   U32 have_cthread:1;     /* the comm thread is running             */
   U32 debug;              /* 1 = write debug messages               */

   int read_ccw_count;
   int write_ccw_count;
   int unack_attn_count;

   BYTE inpbuf[65536];
   int inpbufl;

   void * freeq;
   void * sendq;
   BYTE * poolarea;
 };
 
struct CSW
{
  uint8_t  key;
  uint8_t  data_address[3];
  uint8_t  unit_conditions;
  uint8_t  channel_conditions;
  uint16_t count;
} csw;
/*-------------------------------------------------------------------*/
/* Internal macro definitions                                        */
/*-------------------------------------------------------------------*/

/* DEBUG_LVL: 0 = none
              1 = status
              2 = headers
              3 = buffers
*/
#define DEBUG_LVL        0

#if DEBUG_LVL == 0
  #define TNSDEBUG1      1 ? ((void)0) : logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 1
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 2
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 3
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      logmsg
#endif

#define TNSERROR        logmsg

#define BUFLEN_3270     65536           /* 3270 Send/Receive buffer  */
#define BUFLEN_1052     150             /* 1052 Send/Receive buffer  */


#undef  FIX_QWS_BUG_FOR_MCS_CONSOLES

/*
#ifndef WIN32
  typedef enum { false, true } bool;
#endif
* */

  // **********************************************************
  // Messages
  // **********************************************************

#define ADX00001 "%1d:04X %s  %d %s"
#define ADX00002 "%1d:04X %s"
#define ADX00003 "%1d:04X %s connection to channel adapter on port %d %s"
#define ADX00004 "%1d:04X %s %s"
#define ADX00005 "%1d:04X %s %04X"
#define ADX00006 "%1d:04X %s %02X"
#define ADX00007 "%1d:04X %s %d %s %02X%02X"
#define ADX00008 "%1d:04X %s %d %s %d %s %d"
#define ADX00009 "%1d:04X %s %02X %s %d"
#define ADX00010 "%1d:04X %s %d"
#define ADX00011 "%1d:04X %s %02X %s %02X"
#define ADX00012 "%1d:04X %s %s %s"



  static int write_adpt(BYTE* bufferp, int len, COMMADPT* ca);

// ************************************************************
// Function to check if socket is (still) connected
// ************************************************************
static bool IsSocketConnected(int sockfd, U16 ssid, U16 devnum)
 {
   int rc;
   struct sockaddr_in ccu_addr;
   socklen_t addrlen;
      rc = getpeername(sockfd, (struct sockaddr *)&ccu_addr, &addrlen);
      if (rc == 0)
         return true;
      else {
         //WRMSG(ADX00001, "E",  devnum, "socket", sockfd, strerror(errno));
         logmsg("ADX00001E %1d:%04X: socket %d %s\n",ssid,devnum, sockfd, strerror(errno));
         return false;
      }
   }

/// ********************************************************************
// Function to enable TCP socket and connect to Remote channel adapter
// ********************************************************************
static int connect_adpt(COMMADPT *ca) {
   int rc;
 

  /*struct CUA {
      BYTE caddress;
      BYTE uaddress;
   } cua;
  */
   char cua[2];


   // Bus socket creation
   ca->busfd = socket(AF_INET, SOCK_STREAM, 0);
   if (ca->busfd <= 0 ) {
      //WRMSG( ADX00002, "E",  ca->dev->devnum, "Bus socket creation failed");
      logmsg("ADX00002E %1d:%04X: bus socket creation failed\n", ca->dev->ssid, ca->dev->devnum);
      
      return(-1);
   }

   // ATTN (Tag) socket creation
   ca->tagfd = socket(AF_INET, SOCK_STREAM, 0);
   if (ca->tagfd <= 0) {
      //WRMSG( ADX00002, "E",  ca->dev->devnum, "Tag socket creation failed");
      logmsg("ADX00002E %1d:%04X: tag socket creation failed\n",ca->dev->ssid,ca->dev->devnum);
      return(-1);
   }
   
#ifdef WIN32
   u_long bmode = 0;     // Socket read blocking mode flag
  
   rc = ioctlsocket(ca->busfd, FIONBIO, &bmode);
   if (rc != 0) {
       //WRMSG(ADX00002, "E",  ca->dev->devnum, "Bus socket option failed");
       logmsg("ADX00002E %1d:%04X: bus socket option failed\n", ca->dev->ssid, ca->dev->devnum);
       return(-1);
   }
   rc = ioctlsocket(ca->tagfd, FIONBIO, &bmode);
   if (rc != 0) {
       //WRMSG(ADX00002, "E",  ca->dev->devnum, "Tag socket option failed");
       logmsg("ADX00002E %1d:%04X: tag socket option failed\n", ca->dev->ssid, ca->dev->devnum);
       return(-1);
   }
#endif

   // remove if below works on Linux: bzero(&ca->servaddr, sizeof(ca->servaddr));
   memset(&ca->servaddr, 0x00, sizeof(ca->servaddr));

   // Assign IP addr and PORT number
   ca->servaddr.sin_family = AF_INET;
   ca->servaddr.sin_addr.s_addr = ca->adaptip;
   ca->servaddr.sin_port = htons(ca->port);


   // Connect to the bus socket
   while (connect(ca->busfd, (struct sockaddr*)&ca->servaddr, sizeof(ca->servaddr)) != 0) {
      //WRMSG (ADX00003, "E",  ca->dev->devnum, "bus", ca->busfd, "failed, retry in 10 sec...");
      logmsg("ADX00003E %1d:%04X: bus %d failed, retry in 10 sec...\n",  ca->dev->ssid, ca->dev->devnum, ca->busfd);
      sleep(10);
   }
   // Connect to the tag (ATTN) socket
   while (connect(ca->tagfd, (struct sockaddr*)&ca->servaddr, sizeof(ca->servaddr)) != 0) {
      //WRMSG( ADX00003, "E",  ca->dev->devnum, "tag", ca->tagfd, "failed, retry in 10 sec...");
      logmsg("ADX00003E %1d:%04X: tag %d failed, retry in 10 sec...\n", ca->dev->ssid, ca->dev->devnum, ca->tagfd);
      sleep(10);
   }
   //WRMSG( ADX00003, "I",  ca->dev->devnum, "tag", ca->tagfd, "established");
   logmsg("ADX00003I %1d:%04X: tag connection established on socket %d\n", ca->dev->ssid, ca->dev->devnum, ca->tagfd);

   //cua.caddress = (ca->devnum & 0x0000FF00) >> 8;
   //cua.uaddress = (ca->devnum & 0x000000FF);

    cua[0] = (ca->devnum & 0x0000FF00) >> 8;
    cua[1] = (ca->devnum & 0x000000FF);
   if (ca->busfd  > 0) {
      rc = send (ca->busfd, cua, 2, 0);

       if (rc == 2)
          //WRMSG( ADX00003, "I",  ca->dev->devnum, "bus", ca->busfd, "established");
          logmsg("ADX00003I %1d:%04X: bus connection established on socket %d\n",  ca->dev->ssid, ca->dev->devnum, ca->busfd);
   } else {
      //WRMSG(ADX00004, "E",  ca->dev->devnum, "connect_adpt()", strerror(HSO_errno));
      logmsg("ADX00004E %1d:%04X: connect_adpt() %s\n",  ca->dev->ssid, ca->dev->devnum, strerror(HSO_errno));
      return(-1);
   }

   return(0);
}


/*-------------------------------------------------------------------*/
/* Subroutine to send ack to remote channel adapter                  */
/*-------------------------------------------------------------------*/
static void
send_ack(int sockfd, U16 ssid, U16 devnum) {
   int rc;                              /* Return code               */
   char ackbuf = 0x8F;

   if (sockfd > 0)
      rc = send (sockfd, &ackbuf, 1, 0);
   else
      rc = -1;
   if (rc < 0)
     //WRMSG(ADX00004, "E",  devnum, "send_ack()", strerror(HSO_errno));
     logmsg("ADX00004E %1d:%04X: send_ack() %s\n",  ssid, devnum, strerror(HSO_errno));
   return;
} /* End function send_ack */ 


/*-------------------------------------------------------------------*/
/* Subroutine to receive ack from remote channel adapter             */
/*-------------------------------------------------------------------*/
static void
recv_ack(int sockfd, U16 ssid, U16 devnum) {
   int rc;                             /* Return code               */
   char ackbuf;

   if (sockfd > 0)
      rc = read(sockfd, &ackbuf, sizeof(ackbuf));
   else
      rc = -1;
   if (rc < 0)
      //WRMSG(ADX00004, "E",  devnum, "recv_ack()", strerror(HSO_errno));
      logmsg("ADX00004E %1d:%04X: recv_ack() %s\n",  ssid, devnum, strerror(HSO_errno));
   return;
} /* end function recv_ack */


/*-------------------------------------------------------------------*/
/* Subroutine to send buffer to remote channel adapter               */
/*-------------------------------------------------------------------*/
static int
write_adpt(BYTE *bufferp, int len, COMMADPT *ca) {
   int rc;                              /* Return code               */

   if (ca->busfd > 0)
      rc = send (ca->busfd, bufferp, len, 0);
   else
      rc = -1;
   if (rc < 0) {
      //WRMSG(ADX00004, "E",  ca->dev->devnum, "write_adpt()", strerror(HSO_errno));
      logmsg("ADX00004E %1d:%04X: write_adpt() %s\n",  ca->dev->ssid, ca->dev->devnum, strerror(HSO_errno));
      return -1;
   }
   return 0;
} /* End function write_adpt */


/*-------------------------------------------------------------------*/
/* Subroutine to read from remote channel adapter                    */
/*-------------------------------------------------------------------*/
static int
read_adpt(BYTE *bufferp, COMMADPT *ca) {
   int rc;                              /* Return code               */

   if (ca->busfd > 0)
      rc = read(ca->busfd, bufferp, sizeof(bufferp));
   else
      rc = -1;

   if (rc < 0) {
      //WRMSG(ADX00004, "E",  ca->dev->devnum, "read_adpt()", strerror(HSO_errno));
      logmsg("ADX00004E %1d:%04X: read_ack() %s\n", ca->dev->ssid, ca->dev->devnum, strerror(HSO_errno));
      return -1;
   }
   return rc;
} /* End function read_adpt */


char EBCDIC2ASCII (char s) {
    static char etoa[] =
        "................................"
        "................................"
        " ...........<(+|&.........!$*); "  // first char here is real space
        "-/.........,%_>?.........`:#@'=\""
        " abcdefghi.......jklmnopqr......"
        "  stuvwxyz......................"
        " ABCDEFGHI.......JKLMNOPQR......"
        "  STUVWXYZ......0123456789......";
         s = etoa[(unsigned char)s];
    
   return s; 
}




static void logdump(char *txt,DEVBLK *dev,BYTE *bfr,size_t sz)
{
    size_t i;
    int ru_len;
    uint32_t snacode;
    char *ru_type="";
    if(!dev->ccwtrace)
    {
        return;
    }
    logmsg("HHCCA300D %1d:%04X:%s\n", dev->ssid, dev->devnum, txt);
    logmsg("HHCCA300D %1d:%04X:%s : Dump of %ld (%ld) byte(s)\n", dev->ssid, dev->devnum,txt,sz,sz);
    for(i=0;i<sz;i++)
    {
        if(i%16==0)
        {
            if(i!=0)
            {
                logmsg("\n");
            }
            logmsg("HHCCA300D %1d:%04X:%s : %ld:",dev->ssid, dev->devnum,txt,i);
        }
        if(i%4==0)
        {
            logmsg(" ");
        }
        logmsg("%2.2X",bfr[i]);
    }
    logmsg("\nHHCCA300D ");
    for(i=0;i<sz;i++)
    {
        if(i%16==0)
        {
            if(i!=0)
            {
                logmsg("\nHHCCA300D ");
            }
        }
        logmsg ("%c", EBCDIC2ASCII(bfr[i]));
    }
    logmsg("\n");
    if (strcmp(txt, "WRITE") == 0) {
       if ((bfr[0]  & 0x7C) ==  0x1C) {              //If FID1 single segment
	      ru_len = (bfr[8] << 8) + bfr[9];
	      if (ru_len > 3) {	  
			ru_type = "UNKNOWN";
			if (bfr[13] == 0x11) ru_type = "ACTPU";
			if (bfr[13] == 0x0D) ru_type = "ACTLU";
			if (bfr[13] == 0x0E) ru_type = "DACTLU";
			if (bfr[13] == 0x12) ru_type = "DACTPU";
			if (bfr[13] == 0xA0) ru_type = "SDT";
			if (bfr[13] == 0x31) ru_type = "BIND";
			if (bfr[13] == 0x32) ru_type = "UNBIND";
			if (!memcmp(&bfr[13], "\x01\x02\x01", 3)) ru_type = "CONTACT";
			if (!memcmp(&bfr[13], "\x01\x02\x02", 3)) ru_type = "DISCONTACT";
			if (!memcmp(&bfr[13], "\x01\x02\x03", 3)) ru_type = "IPLINIT";
			if (!memcmp(&bfr[13], "\x01\x02\x04", 3)) ru_type = "IPLTEXT";
			if (!memcmp(&bfr[13], "\x01\x02\x05", 3)) ru_type = "IPLFINAL";
			if (!memcmp(&bfr[13], "\x01\x02\x0A", 3)) ru_type = "ACTLINK";
			if (!memcmp(&bfr[13], "\x01\x02\x0B", 3)) ru_type = "DACTLINK";
  /*  if (!memcmp(&bfr[13], R010211, 3)) {
        sprintf(fmtbuf6, "%s[%02x]", "SETCV", bfr[18]);
        ru_type = fmtbuf6;
        if ((bfr[10] & 0x80) != 0)
            ru_type = "SETCV";
    }
    */
			if (!memcmp(&bfr[13], "\x01\x02\x80", 3)) ru_type = "CONTACTED";
			if (!memcmp(&bfr[13], "\x01\x02\x81", 3)) ru_type = "INOP";
			if (!memcmp(&bfr[13], "\x01\x02\x84", 3)) ru_type = "REQCONT";
			if (!memcmp(&bfr[13], "\x01\x02\x1B", 3)) ru_type = "REQDISCONT";
			if (!memcmp(&bfr[13], "\x01\x02\x1A", 3)) ru_type = "FNA";
			if (!memcmp(&bfr[13], "\x01\x02\x0F", 3)) ru_type = "ABCONN";
			if (!memcmp(&bfr[13], "\x01\x02\x19", 3)) ru_type = "ANA";
			if (!memcmp(&bfr[13], "\x01\x02\x16", 3)) ru_type = "ACTCONNIN";
			if (!memcmp(&bfr[13], "\x01\x02\x17", 3)) ru_type = "DACTCONNIN";
			if ((bfr[10] & 0x08) == 0) ru_type = "";
	         
          }   //End ru_len	
           logmsg("ADX00012D %1d:%04X: Request Unit type: %s\n",  dev->ssid, dev->devnum, ru_type);             
      }   // end FID1
   } // end strcmp1
    
}

static void put_bufpool(void ** anchor, BYTE * ele) {
       void ** elep = anchor;
       for (;;) {
               if (!*elep) break;
               elep = *elep;
       }
       *elep = ele;
       *(void**)ele = 0;
}


static void init_bufpool(COMMADPT *ca) {
        BYTE * areap;
        int i1;
        int numbufs = 64;
        int bufsize = 256+16+4;
        ca->poolarea = (BYTE*)calloc (numbufs, bufsize);
        if (!ca->poolarea) {
                return;
        }
        areap = ca->poolarea;
        for (i1 = 0; i1 < numbufs; i1++) {
                put_bufpool(&ca->freeq, areap);
                areap += (bufsize);
        }
}

static void free_bufpool(COMMADPT *ca) {
        ca->sendq = 0;
        ca->freeq = 0;
        if (ca->poolarea) {
            free(ca->poolarea);
            ca->poolarea = 0;
        }
}

/*-------------------------------------------------------------------*/
/* Free all private structures and buffers                           */
/*-------------------------------------------------------------------*/
static void commadpt_clean_device(DEVBLK *dev)
{
    if(dev->commadpt!=NULL)
    {
        free(dev->commadpt);
        dev->commadpt=NULL;
        if(dev->commadpt->debug)
        {
                logmsg("HHCCA300D %1d:%04X:clean : Control block freed\n", dev->ssid, dev->devnum);
        }
    }
    else
    {
        if(dev->commadpt->debug)
        {
                logmsg("HHCCA300D %1d:%04X:clean : Control block not freed : not allocated\n", dev->ssid, dev->devnum);
        }
    }
    return;
}

/*-------------------------------------------------------------------*/
/* Allocate initial private structures                         */
/*-------------------------------------------------------------------*/
static int commadpt_alloc_device(DEVBLK *dev)
{
    dev->commadpt=malloc(sizeof(COMMADPT));
    if(dev->commadpt==NULL)
    {
        logmsg("HHCCA020E %1d:%04X:Memory allocation failure for main control block\n", dev->ssid, dev->devnum);
        return -1; 
    }
    memset(dev->commadpt,0,sizeof(COMMADPT));
    dev->commadpt->dev=dev;
    return 0;
}

/*-------------------------------------------------------------------*/
/* Parsing utilities                                                 		*/
/*-------------------------------------------------------------------*/
/*-------------------------------------------------------------------*/
/* commadpt_getport : returns a port number or -1                    */
/*-------------------------------------------------------------------*/
static int commadpt_getport(char *txt) {
   int pno;
   struct servent *se;

   pno = atoi(txt);
   if (pno == 0) {
      se = getservbyname(txt, "tcp");
         if (se == NULL) {
            return -1;
         }
      pno = se->s_port;
   }
   return(pno);
}
/*-------------------------------------------------------------------*/
/* commadpt_getaddr : set an in_addr_t if ok, else return -1         */
/*-------------------------------------------------------------------*/
static int commadpt_getaddr(in_addr_t *ia, char *txt) {
   struct hostent *he;
   he = gethostbyname(txt);
   if (he == NULL) {
      return(-1);
   }
   memcpy(ia, he->h_addr_list[0], 4);
   return(0);
}


/*------------------------------------------------------------------------------------------------*/
/* ATTN_adpt: Monitors sockets and handles unsolicited attentions from the remote channel adapter */
/*------------------------------------------------------------------------------------------------*/
static void *ATTN_adpt(void *vca) {
   COMMADPT *ca;
   
   int rc, rc_attn;  /* return code from various rtns     */
    char ackbuf = 0x8F;
    
    
   ca = (COMMADPT*)vca;
  

   while (!IsSocketConnected(ca->busfd, ca->dev->ssid, ca->dev->devnum)) {
      if (ca->dev->commadpt->debug)
         //WRMSG(ADX00002, "D",  ca->dev->devnum, "Preparing connection with remote channel adapter");
         logmsg("ADX00002D %1d:%04X: Preparing connection with remote channel adapter\n", ca->dev->ssid, ca->dev->devnum);

      rc = connect_adpt(ca);

      if (rc == 0) {
         if (ca->dev->commadpt->debug)
            //WRMSG(ADX00008, "D",  ca->dev->devnum, "connections on port ", ca->port, " Bus socket : ", ca->busfd, " Tag socket : ", ca->tagfd);
            logmsg("ADX00008D %1d:%04X:connections on port %d; Bus socket: %d, Tag socket: %d\n", ca->dev->ssid, ca->dev->devnum, ca->port, ca->busfd,ca->tagfd);
      } else {
         ca->dev->scsw.unitstat |= CSW_UC;    // Signal unit check
      }

      while (IsSocketConnected(ca->tagfd, ca->dev->ssid, ca->dev->devnum)) {
         // Wait for Channel Adapter status byte
         rc = recv(ca->tagfd, &ca->carnstat, 1,0);;
         if (rc > 0) {
             if (ca->dev->commadpt->debug)
                 //WRMSG(ADX00006, "D", ca->dev->devnum, "Status received ", ca->carnstat);
                 logmsg("ADX00002D %1d:%04X: Status received %d\n", ca->dev->ssid, ca->dev->devnum, ca->carnstat);

            //if (ca->dev->ccwtrace):
             //   WRMSG(ADX00002, "D", SSID_TO_LCSS(dev->ssid), dev->devnum, "Lock OK");
            ca->unack_attn_count++;
            if (ca->dev->commadpt->debug)
               // WRMSG(ADX00011, "D",  ca->dev->devnum, "Device busy:  ", ca->dev->busy, "CCW active ", ca->ccwactive);
               logmsg("ADX00011D %1d:%04X: Device busy: %02X CCW active %02X \n", ca->dev->ssid, ca->dev->devnum, ca->dev->busy, ca->ccwactive);
            // If a lock is held at this point, we are in a deadlock situation.
            // The attn is skipped to release the lock at the remote channel adapter side.
            if (!test_lock(&ca->lock)) {
               obtain_lock(&ca->lock);
               ca->carnstat |= CSW_ATTN;
               release_lock(&ca->lock);
               rc_attn = device_attention(ca->dev, ca->carnstat);
               obtain_lock(&ca->lock);
               if (rc_attn == 0)
                   if (ca->dev->ccwtrace)
                       //WRMSG(ADX00002, "D",  ca->dev->devnum, "ATTN OK");
                       logmsg("ADX00002D %1d:%04X: ATTN OK\n", ca->dev->ssid, ca->dev->devnum);

               // Acknowledge attention received to the remote channel adapter
               if (ca->tagfd > 0) {
                  ackbuf = 0x8F;            // Indicate ATTN processed
                  rc = send (ca->tagfd, &ackbuf, 1, 0);
                  if (ca->dev->commadpt->debug)
                      //WRMSG(ADX00009, "D",  ca->dev->devnum, "ACK send ", ackbuf, "rc = ",rc);
                      logmsg("ADX00009D %1d:%04X: ACK send %02X, rc = %d\n", ca->dev->ssid, ca->dev->devnum, ackbuf, rc);
               }

               if ((ca->dev->commadpt->debug) && (ca->carnstat & CSW_ATTN))
                  //WRMSG(ADX00010, "D",  ca->dev->devnum, "raised attention, return code ", rc);
                  logmsg("ADX00010D %1d:%04X: raised attention, return code %d\n", ca->dev->ssid, ca->dev->devnum, rc);

               release_lock(&ca->lock);
               if (ca->dev->commadpt->debug)
                   //WRMSG(ADX00002, "D",  ca->dev->devnum, "Release Lock OK");
                   logmsg("ADX00002D %1d:%04X: Release Lock OK\n", ca->dev->ssid, ca->dev->devnum);
            } else {
                if (ca->dev->commadpt->debug)
                    //WRMSG(ADX00002, "D",  ca->dev->devnum, "lock held, cancel ATTN");
                    logmsg("ADX00002D %1d:%04X: Lock held, cancel ATTN\n", ca->dev->ssid, ca->dev->devnum);
               if (ca->tagfd > 0) {
                  ackbuf = 0xF8;   // Indicate ATTN cancelled
                  rc = send (ca->tagfd, &ackbuf, 1, 0);
                  if (ca->dev->commadpt->debug)
                      //WRMSG(ADX00009, "D",  ca->dev->devnum, "ACK cancelled ", ackbuf, "rc = ", rc);
                      logmsg("ADX00009D %1d:%04X: ACK cancelled %02X, rc = %d\n", ca->dev->ssid, ca->dev->devnum, ackbuf, rc);
               }
            }

            // Error during receive of CA status.
            // Close both sockets and try to re-open
            if (rc == -1) {
                if (ca->dev->commadpt->debug)
                    //WRMSG(ADX00002, "D",  ca->dev->devnum, "Closing sockets due to error");
                    logmsg("ADX00002D %1d:%04X: Closing sockets due to error\n", ca->dev->ssid, ca->dev->devnum);
               close(ca->busfd);
               close(ca->tagfd);
               ca->dev->scsw.unitstat |= CSW_UC;    //Signal unit check
               sleep(10);   /* wait 10 secs, then try to re-connect */
            }  // End rc == -1
         } else { // if rc > 0
            if (ca->dev->commadpt->debug)
                //WRMSG(ADX00002, "D",  ca->dev->devnum, "Closing sockets due to error");
                logmsg("ADX00002D %1d:%04X: Closing sockets due to error\n", ca->dev->ssid, ca->dev->devnum);
            close(ca->busfd);
            close(ca->tagfd);
            ca->dev->scsw.unitstat |= CSW_UC;    //Signal unit check
            sleep(10);   /* wait 10 secs, then try to re-connect */
         } // End if rc > 0
      }  // End while ca->tagfd
   }  // End while ca->busfd
   return NULL;
}

/*-------------------------------------------------------------------*/
/* Halt currently executing I/O command                              */
/*-------------------------------------------------------------------*/
static void    commadpt_halt(DEVBLK *dev)
{
    if(!dev->busy)
    {
        return;
    }
}

/* The following 3 MSG functions ensure only 1 (one)  */
/* hardcoded instance exist for the same numbered msg */
/* that is issued on multiple situations              */
static void msg013e(DEVBLK *dev,char *kw,char *kv)
{
        logmsg("HHCCA013E %1d:%04X:Incorrect %s specification %s\n",dev->ssid, dev->devnum,kw,kv);
}


/*-------------------------------------------------------------------*/
/* Device Initialisation                                             */
/*-------------------------------------------------------------------*/
static int commadpt_init_handler (DEVBLK *dev, int argc, char *argv[]) {
   char thread_name[32];
   int i;
   int rc;
   int pc; /* Parse code */
   int errcnt;
   union {
      int num;
      char text[80];  // Version 3
      // Version 4: char text[MAX_PARSER_STRLEN+1];
   } res;



   /* For re-initialisation, close the existing file, if any */
   if (dev->fd >= 0)
      (dev->hnd->close)(dev);

   dev->devtype = 0x3705;
  // Version 4: dev->excps = 0;
      //WRMSG(HHC01058, "I", SSID_TO_LCSS(dev->ssid), dev->devnum);
      logmsg("HHC01058I %1d:%04X: initialization starting\n", dev->ssid, dev->devnum); 
         
   /* legacy sense-id not supported */
   dev->numdevid = 0;

   if (dev->commadpt != NULL) {
      commadpt_clean_device(dev);
   }
   rc = commadpt_alloc_device(dev);

   if (rc < 0) {
      //WRMSG(HHC01011, "I", SSID_TO_LCSS(dev->ssid), dev->devnum);
      logmsg("HHC01011I %1d:%04X: initialization not performed\n",dev->ssid, dev->devnum);
      return(-1);
   }

    //WRMSG(HHC01059, "E", SSID_TO_LCSS(dev->ssid), dev->devnum);
    logmsg("HHC01059E %1d:%04X: initialization: control block allocated\n",dev->ssid, dev->devnum);

   errcnt = 0;

   /* Initialise ports & hosts */
   dev->commadpt->busfd = -1;
   dev->commadpt->tagfd = -1;
   dev->commadpt->port = 0;

     for(i=0;i<argc;i++)
           {  
      
        pc = parser(ptab, argv[i], &res);

           
        if (pc < 0) {
           //WRMSG(HHC01012, "E", SSID_TO_LCSS(dev->ssid), dev->devnum, argv[i]);
           logmsg("HHC01012E %1d:%04X: error parsing %s\n", dev->ssid, dev->devnum,argv[i]);
           errcnt++;
           continue;
         }   
        if (pc == 0) {
           //WRMSG(HHC01019, "E", SSID_TO_LCSS(dev->ssid), dev->devnum, argv[i]);
           logmsg("HHC01019E %1d:%04X:  unrecognized parameter %s\n",dev->ssid, dev->devnum, argv[i]);
            errcnt++;
            continue;
         }

         switch(pc) {
            case COMMADPT_KW_DEBUG:
		       if (res.text[0] == 'y' || res.text[0] == 'Y')
			      dev->commadpt->debug = 1;
		       else
			      dev->commadpt->debug = 0;  
               break;
            case COMMADPT_KW_PORT:
               rc = commadpt_getport(res.text);
               if (rc < 0) {
                  errcnt++;
                  msg013e(dev, "PORT", res.text);
                  break;
               }
               dev->commadpt->port = rc;
               break;
            case COMMADPT_KW_ADAPTIP:
               if (strcmp(res.text, "*") == 0) {
                  dev->commadpt->adaptip = INADDR_ANY;
                  break;
               }
               rc = commadpt_getaddr(&dev->commadpt->adaptip, res.text);
               if (rc != 0) {
                  msg013e(dev, "ADAPTIP", res.text);
                  errcnt++;
               }
               break;
            default:
               break;
      }  // End of switch(pc)
   }   // End of for stmt.
   

   if (errcnt > 0) {
      //WRMSG(HHC01014, "I", SSID_TO_LCSS(dev->ssid), dev->devnum);
      logmsg("HHC01014I %1d:%04X: initialization failed due to previous errors", dev->ssid, dev->devnum);
      
      return -1;
   }

   dev->bufsize = 256;
   dev->numsense = 2;
   memset(dev->sense, 0, sizeof(dev->sense));
   


   init_bufpool(dev->commadpt);

   dev->commadpt->devnum = dev->devnum;

   /* Initialize the CA lock */
   initialize_lock(&dev->commadpt->lock);

   /* Initialise thread->I/O & halt initiation EVB */
   initialize_condition(&dev->commadpt->ipc);
   initialize_condition(&dev->commadpt->ipc_halt);

   /* Allocate I/O -> Thread signaling pipe */
   VERIFY(!create_pipe(dev->commadpt->pipe));

   /* Obtain the CA lock */
   obtain_lock(&dev->commadpt->lock);

   /* Start the thread to establish client connections with the remote channel adapter */
    strcpy(thread_name,"ATTN_adpt "); 

   /* Set thread-name for debugging purposes */
    if (dev->commadpt->debug)
               // WRMSG(ADX00012, "D",  ca->dev->devnum, "Starting thread ", thread_name);
               logmsg("ADX00012D %1d:%04X: Starting thread %s \n", dev->ssid, dev->devnum,thread_name);

//   thread_name[sizeof(thread_name)-1] = 0;

   rc = create_thread(&dev->commadpt->tthread, &sysblk.detattr, ATTN_adpt, dev->commadpt, thread_name);
   if (rc) {
      //WRMSG(HHC00102, "E" , strerror(rc));
      logmsg("HHC00102E %1d:%04X: Error in function create_thread(): %s\n", dev->ssid, dev->devnum,strerror(rc));
      release_lock(&dev->commadpt->lock);
      return -1;
   }

   dev->commadpt->have_cthread = 1;

   /* Release the CA lock */
   release_lock(&dev->commadpt->lock);
   /* Indicate succesfull completion */
   return 0;
}


/*-------------------------------------------------------------------*/
/* Query the device definition                                       */
/*-------------------------------------------------------------------*/
static void commadpt_query_device (DEVBLK *dev, char **class,
                int buflen, char *buffer)
{
    *class = "LINE";
    snprintf(buffer,buflen,"Read count=%d, Write count=%d\n", dev->commadpt->read_ccw_count, dev->commadpt->write_ccw_count);
}

/*-------------------------------------------------------------------*/
/* Close the device                                                  */
/* Invoked by HERCULES shutdown & DEVINIT processing                 */
/*-------------------------------------------------------------------*/
static int commadpt_close_device ( DEVBLK *dev )
{
    if(dev->ccwtrace)
    {
        logmsg("HHCCA300D %1d:%04X:Closing down\n", dev->ssid, dev->devnum);
    }

    /* Obtain the CA lock */
    obtain_lock(&dev->commadpt->lock);

    /* Terminate current I/O thread if necessary */
    if(dev->busy)
    {
        commadpt_halt(dev);
    }

    free_bufpool(dev->commadpt);

    /* release the CA lock */
    release_lock(&dev->commadpt->lock);

    /* Free all work storage */
    commadpt_clean_device(dev);

    /* Indicate to hercules the device is no longer opened */
    dev->fd=-1;

    if(dev->ccwtrace)
    {
        logmsg("HHCCA300D %1d:%04X:Closed down\n", dev->ssid, dev->devnum);
    }
    return 0;
}



/*-------------------------------------------------------------------*/
/* The Below subroutine is replaced by the channel adapter extender  */
/*                                                                   */
/*-------------------------------------------------------------------*/
/*-------------------------------------------------------------------*/
/* Execute a Channel Command Word                                    */
/*-------------------------------------------------------------------*/
static void commadpt_execute_ccw (DEVBLK *dev, BYTE code, BYTE flags,
        BYTE chained, U32 count, BYTE prevcode, int ccwseq,
        BYTE *iobuf, BYTE *more, BYTE *unitstat, U32 *residual)
   {
   U32 num;                        /* Work : Actual CCW transfer count                   */
 //  BYTE    *piudata;
//   int     piusize;
 //  void    *eleptr;
 //  int     llsize;
   char    buf[80];
   int     rc;
 //  char    ackbuf[3];
   char CCW_Code[2];


   struct CCW {
      BYTE code;
      BYTE data_address[3];
      BYTE flags;
      BYTE chain;
      BYTE count_ho;
      BYTE count_lo;
      } ccw;

   ccw.code = code;
   ccw.flags = flags;
   ccw.chain = chained;
   ccw.count_ho = (count & 0x0000FF00) >> 8 ;
   ccw.count_lo = (count & 0x000000FF);

   if (dev->commadpt->debug)
       //WRMSG(ADX00005, "D", SSID_TO_LCSS(dev->ssid), dev->devnum, "ccw count=", count);
       logmsg("ADX00005D %1d:%04X: CCW=%02X, CCW count=%04X\n", dev->ssid, dev->devnum, code, count);


   UNREFERENCED(flags);
   UNREFERENCED(chained);
   UNREFERENCED(prevcode);
   UNREFERENCED(ccwseq);

   *residual = 0;

   if (IsSocketConnected(dev->commadpt->busfd, dev->ssid, dev->devnum)) {
	    /* Obtain the COMMADPT lock */
      obtain_lock(&dev->commadpt->lock);
      dev->commadpt->ccwactive = 0x01;
      CCW_Code[0] = 0x00;
      CCW_Code[1] = code;
      if (dev->commadpt->debug)
          //WRMSG(ADX00006, "D", SSID_TO_LCSS(dev->ssid), dev->devnum, "Sending CCW", code);
          logmsg("ADX00006D %1d:%04X: Sending CCW %02X\n", dev->ssid ,dev->devnum, code);

      rc = write_adpt((void*)&ccw, sizeof(ccw), dev->commadpt);
      /* wait for ACK */
      recv_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);

      switch (code) {
         /*---------------------------------------------------------------*/
         /* Test I/O                                                      */
         /*---------------------------------------------------------------*/
         case 0x00:
             *residual = 0;
             break;

         /*---------------------------------------------------------------*/
         /* I/O NO-OP                                                     */
         /*---------------------------------------------------------------*/
         case 0x03:
             *residual = count;
             /* Get Channel Return Status */
             rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
             *unitstat = dev->commadpt->carnstat;
             /* send ACK */
             send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
             break;

         /*---------------------------------------------------------------*/
         /* BASIC SENSE                                                   */
         /*---------------------------------------------------------------*/
         case 0x04:
            /* Wait for the sense data */
            rc = read(dev->commadpt->busfd, dev->sense, 256);
            dev->numsense = rc;
            dev->commadpt->unack_attn_count = 0;
            num = count<dev->numsense?count:dev->numsense;
            *more = count<dev->numsense?1:0;
            if (dev->commadpt->debug)
               //WRMSG(ADX00007, "D", SSID_TO_LCSS(dev->ssid), dev->devnum,"Received", rc, "sense data ", dev->sense[0], dev->sense[1]);
               logmsg("ADX00007D %1d:%04X: sense data %02X%02X\n",dev->ssid, dev->devnum, dev->sense[0], dev->sense[1]);
            /* Copy device sense bytes to channel I/O buffer */
            memcpy (iobuf, dev->sense, rc);
            *residual = count-num;
            /* Send the ACK */
            send_ack(dev->commadpt->busfd, dev->ssid,dev->devnum);
            /* Read the CSW */
            rc = read(dev->commadpt->busfd, dev->commadpt->inpbuf, 256);
            // Below  is a bypass. 3705/miniROS sends x00 as the initial CSW flags, which halts further I/O
            // *unitstat = dev->commadpt->inpbuf[4];     // write CSW
            *unitstat = CSW_CE | CSW_DE;
            /* Send the ACK */
            send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            break;

         /*---------------------------------------------------------------*/
         /* Write IPL                                                     */
         /*---------------------------------------------------------------*/
         case 0x05:
            dev->commadpt->unack_attn_count = 0;
            logdump("WRITE", dev, iobuf, count);
            rc = write_adpt(iobuf, count, dev->commadpt);
            /* Wait for the ACK from the remote channel adapter */
            recv_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            if (rc == 0) {
               *residual = 0;
               /* Get Channel Return Status */
               rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
               *unitstat = dev->commadpt->carnstat;
               /* send ACK */
               send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            } else {
               *unitstat|= CSW_ATTN;
               *unitstat|= CSW_UX|CSW_ATTN;
            }
            break;
         /*---------------------------------------------------------------*/
         /* Write start 0                                                 */
         /* Write start 1                                                 */
         /* Read  start 0                                                 */
         /* Read  start 1                                                 */
         /* Reset Restart                                                 */
         /*---------------------------------------------------------------*/
         case 0x31:
         case 0x32:
         case 0x51:
         case 0x52:
         case 0x93:
            dev->commadpt->unack_attn_count = 0;
            /* Get Channel Return Status */
            rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
            *residual = count;
            *unitstat = dev->commadpt->carnstat;
            /* send ACK */
            send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            break;

         /*---------------------------------------------------------------*/
         /* READ CCW                                                      */
         /*---------------------------------------------------------------*/
         case 0x02:     /* READ */
            /* Wait for the remote channel adapter data */
            rc = read(dev->commadpt->busfd, dev->commadpt->inpbuf, count);

            dev->commadpt->read_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            *more = 0;
            /* Copy data to I/O buffer */
            memcpy (iobuf, dev->commadpt->inpbuf, rc);
            *residual = count - rc;
            logdump("READ", dev, iobuf, rc);
            /* Send the ACK */
            send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);

            /* Get Channel Return Status */
            rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
            *unitstat = dev->commadpt->carnstat;
            /* send ACK */
            send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
#if 0  //  <=== !!!
            if (dev->commadpt->sendq) {
               *unitstat|= CSW_ATTN;
            }
#endif
            break;

         /*---------------------------------------------------------------*/
         /* WRITE CCW                                                     */
         /*---------------------------------------------------------------*/
         case 0x01:     /* WRITE */
         case 0x09:     /* WRITE BREAK */
            dev->commadpt->write_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            logdump("WRITE", dev, iobuf, count);

            rc = write_adpt(iobuf, count, dev->commadpt);
            /* Wait for the ACK from the remote channel adapter */
            recv_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            if (rc == 0) {
               *residual = 0;
               /* Get Channel Return Status */
               rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
               *unitstat = dev->commadpt->carnstat;
               /* Send ACK */
               send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            } else {
               *unitstat|= CSW_ATTN;
               *unitstat|= CSW_UX|CSW_ATTN;
            }
#if 0  //  <=== !!!
            if (dev->commadpt->sendq) {
                *unitstat|= CSW_ATTN;
                *unitstat|= CSW_UX|CSW_ATTN;
            }
#endif
            break;

         /*---------------------------------------------------------------*/
         /* All other CCWs: get sense byte from device */
         /*---------------------------------------------------------------*/
         default:

            /* Wait for the sense data */
            rc = read(dev->commadpt->busfd, dev->sense, 256);
            dev->numsense = rc;
            dev->commadpt->unack_attn_count = 0;
            num = count<dev->numsense?count:dev->numsense;
            *more = count<dev->numsense?1:0;
            if (dev->commadpt->debug)
               //WRMSG(ADX00007, "D", SSID_TO_LCSS(dev->ssid), dev->devnum,"Received", rc, "sense data ", dev->sense[0], dev->sense[1]);
               logmsg("ADX00007D %1d:%04X: sense data %02X%02X\n",dev->ssid, dev->devnum, dev->sense[0], dev->sense[1]);
            /* Copy device sense bytes to channel I/O buffer */
            memcpy (iobuf, dev->sense, rc);
            *residual = count-num;
            /* Send the ACK */
            send_ack(dev->commadpt->busfd, dev->ssid,dev->devnum);
             /* Get Channel Return Status */
             rc = read(dev->commadpt->busfd, &dev->commadpt->carnstat, 1);
             *unitstat = dev->commadpt->carnstat;
            /* Send the ACK */
            send_ack(dev->commadpt->busfd, dev->ssid, dev->devnum);
            break;

  }
     // remove if below works under Linux: bzero(buf, 80);
     memset(buf, 0x00, 80);
     dev->commadpt->ccwactive = 0x00;
     release_lock(&dev->commadpt->lock);
  } else {
	     /*---------------------------------------------------------------*/
        /*  3705 not online, set command reject sense 					  */
        /*----------------------------------------------------------------*/
            /* Set command reject sense byte, and unit check status */
            if (code == 0xE4) {
            *unitstat=CSW_CE+CSW_DE+CSW_UC;
            dev->sense[0]=SENSE_CR;
		}
		else
			*unitstat=CSW_CE+CSW_DE;
		
  } // end if busfd != -1

}



/*---------------------------------------------------------------*/
/* DEVICE FUNCTION POINTERS                                      */
/*---------------------------------------------------------------*/

#if defined(OPTION_DYNAMIC_LOAD)
static
#endif
DEVHND com3705_device_hndinfo = {
        &commadpt_init_handler,        /* Device Initialisation      */
        &commadpt_execute_ccw,         /* Device CCW execute         */
        &commadpt_close_device,        /* Device Close               */
        &commadpt_query_device,        /* Device Query               */
        NULL,                          /* Device Start channel pgm   */
        NULL,                          /* Device End channel pgm     */
        NULL,                          /* Device Resume channel pgm  */
        NULL,                          /* Device Suspend channel pgm */
        NULL,                          /* Device Read                */
        NULL,                          /* Device Write               */
        NULL,                          /* Device Query used          */
        NULL,                          /* Device Reserve             */
        NULL,                          /* Device Release             */
        NULL,                          /* Device Attention           */
        commadpt_immed_command,        /* Immediate CCW Codes        */
        NULL,                          /* Signal Adapter Input       */
        NULL,                          /* Signal Adapter Output      */
        NULL,                          /* Hercules suspend           */
        NULL                           /* Hercules resume            */
};


/* Libtool static name colision resolution */
/* note : lt_dlopen will look for symbol & modulename_LTX_symbol */
#if !defined(HDL_BUILD_SHARED) && defined(HDL_USE_LIBTOOL)
#define hdl_ddev hdt3705_LTX_hdl_ddev
#define hdl_depc hdt3705_LTX_hdl_depc
#define hdl_reso hdt3705_LTX_hdl_reso
#define hdl_init hdt3705_LTX_hdl_init
#define hdl_fini hdt3705_LTX_hdl_fini
#endif


HDL_DEPENDENCY_SECTION;
{
     HDL_DEPENDENCY(HERCULES);
     HDL_DEPENDENCY(DEVBLK);
     HDL_DEPENDENCY(SYSBLK);
}
END_DEPENDENCY_SECTION;


#if defined(WIN32) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
  #undef sysblk
  HDL_RESOLVER_SECTION;
  {
    HDL_RESOLVE_PTRVAR( psysblk, sysblk );
  }
  END_RESOLVER_SECTION;
#endif


HDL_DEVICE_SECTION;
{
    HDL_DEVICE(3705, com3705_device_hndinfo );
}
END_DEVICE_SECTION;










