/****************************************************************************
 * net/devspecsock/devspecsock_conn.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_DEV_SPEC_SOCK)

#include <stdint.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "devspecsock/devspecsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all devspecsock connections. */

struct devspecsock_conn_s g_devspecsock_connections[CONFIG_NET_DEV_SPEC_SOCK_CONNS];

/* A list of all free devspecsock connections */

static dq_queue_t g_free_devspecsock_connections;
static sem_t g_free_sem;

/* A list of all allocated devspecsock connections */

static dq_queue_t g_active_devspecsock_connections;

/* The lower socket interface */

static FAR struct devspecsock_sockif_s *g_devspec_lowersockif;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _devspecsock_semtake() and _devspecsock_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static void _devspecsock_semtake(FAR sem_t *sem)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = net_lockedwait(sem)) < 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(ret == -EINTR);
    }
}

static void _devspecsock_semgive(FAR sem_t *sem)
{
  (void)sem_post(sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devspecsock_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized devspecsock connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct devspecsock_conn_s *devspecsock_alloc(void)
{
  FAR struct devspecsock_conn_s *conn;

  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  _devspecsock_semtake(&g_free_sem);
  conn = (FAR struct devspecsock_conn_s *)dq_remfirst(&g_free_devspecsock_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      memset(conn, 0, sizeof(*conn));

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->node, &g_active_devspecsock_connections);
    }

  _devspecsock_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: devspecsock_free()
 *
 * Description:
 *   Free a devspecsock connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void devspecsock_free(FAR struct devspecsock_conn_s *conn)
{
  /* The free list is only accessed from user, non-interrupt level and
   * is protected by a semaphore (that behaves like a mutex).
   */

  DEBUGASSERT(conn->crefs == 0);

  _devspecsock_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_devspecsock_connections);

  /* Reset structure */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_devspecsock_connections);
  _devspecsock_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: devspecsock_initialize()
 *
 * Description:
 *   Initialize the Device-specific Socket connection structures.
 *   Called once and only from the networking layer.
 *
 ****************************************************************************/

void devspecsock_initialize(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_devspecsock_connections);
  dq_init(&g_active_devspecsock_connections);
  sem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_DEV_SPEC_SOCK_CONNS; i++)
    {
      FAR struct devspecsock_conn_s *conn = &g_devspecsock_connections[i];

      /* Mark the connection closed and move it to the free list */

      memset(conn, 0, sizeof(*conn));
      dq_addlast(&conn->node, &g_free_devspecsock_connections);
    }

  g_devspec_lowersockif = NULL;
}

/****************************************************************************
 * Name: devspecsock_sockif
 *
 * Description:
 *   Return the socket interface.
 *
 ****************************************************************************/

FAR struct devspecsock_sockif_s *devspecsock_sockif(void)
{
  return g_devspec_lowersockif;
}

/****************************************************************************
 * Name: devspecsock_register
 *
 * Description:
 *   Register device-specific socket interface.
 *
 ****************************************************************************/

int devspecsock_register(FAR struct devspecsock_sockif_s *sockif)
{
  if (g_devspec_lowersockif)
    {
      return -EEXIST;
    }

  g_devspec_lowersockif = sockif;

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
