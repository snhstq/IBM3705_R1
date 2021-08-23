/*-------------------------------------------------------------------*/
/* Macro definitions for thread functions                            */
/*-------------------------------------------------------------------*/

#include <pthread.h>
typedef pthread_t          TID;
typedef pthread_mutex_t    LOCK;
typedef pthread_cond_t     COND;
typedef pthread_attr_t     ATTR;
#define initialize_lock(plk) \
        pthread_mutex_init((plk),NULL)
#define obtain_lock(plk) \
        pthread_mutex_lock((plk))
#define release_lock(plk) \
        pthread_mutex_unlock((plk))
#define initialize_condition(pcond) \
        pthread_cond_init((pcond),NULL)
#define signal_condition(pcond) \
        pthread_cond_broadcast((pcond))
#define wait_condition(pcond,plk) \
        pthread_cond_wait((pcond),(plk))
#define initialize_detach_attr(pat) \
        pthread_attr_init((pat)); \
        pthread_attr_setdetachstate((pat),PTHREAD_CREATE_DETACHED)
typedef void*THREAD_FUNC(void*);
#define create_thread(ptid,pat,fn,arg) \
        pthread_create(ptid,pat,(THREAD_FUNC*)&fn,arg)
#define signal_thread(tid,signo) \
        pthread_kill(tid,signo)
#define thread_id() \
        pthread_self()

