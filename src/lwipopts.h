#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H
#define LWIP_SOCKETS                   1
#define LWIP_SO_RCVBUF                 1
#define LWIP_BUFSIZE_DEFAULT           256


#define TCPIP_THREAD_PRIO              2
#define TCPIP_THREAD_STACKSIZE         2048
#define DEFAULT_THREAD_STACKSIZE       1024
// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition
#include "lwipopts_examples_common.h"

#endif

