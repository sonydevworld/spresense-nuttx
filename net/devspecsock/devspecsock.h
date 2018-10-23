/****************************************************************************
 * net/devspecsock/devspecsock.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NET_DEVSPECSOCK_DEVSPECSOCK_H
#define __NET_DEVSPECSOCK_DEVSPECSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_DEV_SPEC_SOCK

#include <sys/types.h>
#include <queue.h>
#include <semaphore.h>

#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SOCK_DEVSPECSOCK_TYPE   0x7e

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct devspecsock_sockif_s
{
  /* Socket interface */

  FAR const struct sock_intf_s *sockif;

#if defined(CONFIG_NET_SOCKOPTS)
  /* get/setsockopt interface */

  CODE int (*si_getsockopt)(FAR struct socket *psock, int level, int option,
                            FAR void *value, FAR socklen_t *value_len);
  CODE int (*si_setsockopt)(FAR struct socket *psock, int level, int option,
                            FAR const void *value, FAR socklen_t value_len);
#endif /* CONFIG_NET_SOCKOPTS */
};

struct devspecsock_conn_s
{
  dq_entry_t  node;                   /* Supports a doubly linked list */
  uint8_t     crefs;                  /* Reference counts on this instance */
  uint8_t     s_type;                 /* Protocol type */

  FAR void   *devspec_conn; /* Connection: device specific structure */

  /* Device-specific Socket interface */

  FAR const struct devspecsock_sockif_s *lower_sockif;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

EXTERN const struct sock_intf_s g_devspecsock_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: devspecsock_initialize()
 *
 * Description:
 *   Initialize the Device-specific Socket connection structures.
 *   Called once and only from the networking layer.
 *
 ****************************************************************************/

void devspecsock_initialize(void);

/****************************************************************************
 * Name: devspecsock_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized devspecsock connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct devspecsock_conn_s *devspecsock_alloc(void);

/****************************************************************************
 * Name: devspecsock_free()
 *
 * Description:
 *   Free a devspecsock connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void devspecsock_free(FAR struct devspecsock_conn_s *conn);

/****************************************************************************
 * Name: devspecsock_sockif
 *
 * Description:
 *   Return the socket interface.
 *
 * Parameters:
 *   none
 *
 * Returned Value:
 *   On success, a non-NULL instance of struct sock_intf_s is returned.
 *   NULL is returned if device-specific socket interface is not registered.
 *
 ****************************************************************************/

FAR struct devspecsock_sockif_s *devspecsock_sockif(void);

/****************************************************************************
 * Name: devspecsock_register
 *
 * Description:
 *   Register device-specific socket interface.
 *
 * Parameters:
 *   sockif  device-specific socket interface
 *
 * Returned Value:
 *   On success, 0 is returned.
 *   -EEXIST is returned if device-specific socket interface has
 *   already registered.
 *
 ****************************************************************************/

int devspecsock_register(FAR struct devspecsock_sockif_s *sockif);

#if defined(CONFIG_NET_SOCKOPTS)

/****************************************************************************
 * Name: devspecsock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int devspecsock_getsockopt(FAR struct socket *psock, int level, int option,
                           FAR void *value, FAR socklen_t *value_len);

/****************************************************************************
 * Name: devspecsock_setsockopt
 *
 * Description:
 *   psock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket on the 'psock' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int devspecsock_setsockopt(FAR struct socket *psock, int level, int option,
                           FAR const void *value, FAR socklen_t value_len);

#endif /* CONFIG_NET_SOCKOPTS */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_DEV_SPEC_SOCK */
#endif /* __NET_DEVSPECSOCK_DEVSPECSOCK_H */
