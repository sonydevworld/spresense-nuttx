/****************************************************************************
 * net/devspecsock/devspecsock_sockif.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "devspecsock/devspecsock.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        devspecsock_sockif_setup(FAR struct socket *psock,
                                           int protocol);
static sockcaps_t devspecsock_sockif_sockcaps(FAR struct socket *psock);
static void       devspecsock_sockif_addref(FAR struct socket *psock);
static int        devspecsock_sockif_bind(FAR struct socket *psock,
                                          FAR const struct sockaddr *addr,
                                          socklen_t addrlen);
static int        devspecsock_sockif_getsockname(FAR struct socket *psock,
                                                 FAR struct sockaddr *addr,
                                                 FAR socklen_t *addrlen);
static int        devspecsock_sockif_listen(FAR struct socket *psock,
                                            int backlog);
static int        devspecsock_sockif_connect(FAR struct socket *psock,
                                             FAR const struct sockaddr *addr,
                                             socklen_t addrlen);
static int        devspecsock_sockif_accept(FAR struct socket *psock,
                                            FAR struct sockaddr *addr,
                                            FAR socklen_t *addrlen,
                                            FAR struct socket *newsock);
#ifndef CONFIG_DISABLE_POLL
static int        devspecsock_sockif_poll(FAR struct socket *psock,
                                          FAR struct pollfd *fds, bool setup);
#endif
static ssize_t    devspecsock_sockif_send(FAR struct socket *psock,
                                          FAR const void *buf, size_t len,
                                          int flags);
static ssize_t    devspecsock_sockif_sendto(FAR struct socket *psock,
                                            FAR const void *buf, size_t len,
                                            int flags,
                                            FAR const struct sockaddr *to,
                                            socklen_t tolen);
#ifdef CONFIG_NET_SENDFILE
static ssize_t    devspecsock_sockif_sendfile(FAR struct socket *psock,
                                              FAR struct file *infile,
                                              FAR off_t *offset,
                                              size_t count)
  devspecsock_sockif_sendfile,    /* si_sendfile */
#endif
static ssize_t    devspecsock_sockif_recvfrom(FAR struct socket *psock,
                                              FAR void *buf, size_t len,
                                              int flags,
                                              FAR struct sockaddr *from,
                                              FAR socklen_t *fromlen);
static int        devspecsock_sockif_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_devspecsock_sockif =
{
  devspecsock_sockif_setup,       /* si_setup */
  devspecsock_sockif_sockcaps,    /* si_sockcaps */
  devspecsock_sockif_addref,      /* si_addref */
  devspecsock_sockif_bind,        /* si_bind */
  devspecsock_sockif_getsockname, /* si_getsockname */
  devspecsock_sockif_listen,      /* si_listen */
  devspecsock_sockif_connect,     /* si_connect */
  devspecsock_sockif_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  devspecsock_sockif_poll,        /* si_poll */
#endif
  devspecsock_sockif_send,        /* si_send */
  devspecsock_sockif_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  devspecsock_sockif_sendfile,    /* si_sendfile */
#endif
  devspecsock_sockif_recvfrom,    /* si_recvfrom */
  devspecsock_sockif_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devspecsock_sockif_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Parameters:
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int devspecsock_sockif_setup(FAR struct socket *psock, int protocol)
{
  int                              ret;
  FAR struct devspecsock_sockif_s *lower_sockif;
  FAR struct devspecsock_conn_s   *conn;

  /* Get socket interface */

  lower_sockif = devspecsock_sockif();

  if (lower_sockif && lower_sockif->sockif)
    {
      conn = devspecsock_alloc();
      if (!conn)
        {
          /* Failed to reserve a connection structure */

          ret = -ENOMEM;
          goto errout;
        }

      conn->s_type  = psock->s_type;
      psock->s_type = SOCK_DEVSPECSOCK_TYPE;
      psock->s_conn = conn;

      ret = lower_sockif->sockif->si_setup(psock, protocol);
      if (ret < 0)
        {
          psock->s_type = conn->s_type;
          psock->s_conn = NULL;
          goto errout_free;
        }

      conn->lower_sockif = lower_sockif;
      conn->crefs        = 1;
    }
  else
    {
      return -ENETDOWN;
    }

  return OK;

errout_free:
  devspecsock_free(conn);

errout:
  return ret;
}

/****************************************************************************
 * Name: devspecsock_sockif_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t devspecsock_sockif_sockcaps(FAR struct socket *psock)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_sockcaps != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_sockcaps(psock);
}

/****************************************************************************
 * Name: devspecsock_sockif_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void devspecsock_sockif_addref(FAR struct socket *psock)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_addref != NULL);

  sockif = conn->lower_sockif->sockif;

  sockif->si_addref(psock);

  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: devspecsock_sockif_bind
 *
 * Description:
 *   bind() gives the socket 'conn' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long. Traditionally, this is called
 *   "assigning a name to a socket." When a socket is created with socket,
 *   it exists in a name space (address family) but has no name assigned.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket to be bound
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 ****************************************************************************/

static int devspecsock_sockif_bind(FAR struct socket *psock,
                                   FAR const struct sockaddr *addr,
                                   socklen_t addrlen)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_bind != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_bind(psock, addr, addrlen);
}

/****************************************************************************
 * Name: devspecsock_sockif_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound
 *   name of the specified socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   psock    A reference to the socket structure
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

static int devspecsock_sockif_getsockname(FAR struct socket *psock,
                                          FAR struct sockaddr *addr,
                                          FAR socklen_t *addrlen)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_getsockname != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_getsockname(psock, addr, addrlen);
}

/****************************************************************************
 * Name: devspecsock_sockif_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of AFINET
 *   and AFINET6 sockets, psock_listen() calls this function.  The
 *   psock_listen() call applies only to sockets of type SOCK_STREAM or
 *   SOCK_SEQPACKET.
 *
 * Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See list() for the set of appropriate error values.
 *
 ****************************************************************************/

int devspecsock_sockif_listen(FAR struct socket *psock, int backlog)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_listen != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_listen(psock, backlog);
}

/****************************************************************************
 * Name: devspecsock_sockif_connect
 *
 * Description:
 *   devspecsock_sockif_connect() connects the local socket referred to
 *   by the structure 'psock' to the address specified by 'addr'.
 *   The addrlen argument specifies the size of 'addr'.
 *   The format of the address in 'addr' is determined by the address space
 *   of the socket 'psock'.
 *
 *   If the socket 'psock' is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully
 *   inet_connect() only once; connectionless protocol sockets may use
 *   inet_connect() multiple times to change their association.
 *   Connectionless sockets may dissolve the association by connecting to
 *   an address with the sa_family member of sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failue.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int devspecsock_sockif_connect(FAR struct socket *psock,
                                      FAR const struct sockaddr *addr,
                                      socklen_t addrlen)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_connect != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_connect(psock, addr, addrlen);
}

/****************************************************************************
 * Name: devspecsock_sockif_accept
 *
 * Description:
 *   The accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an inet_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, inet_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, inet_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int devspecsock_sockif_accept(FAR struct socket *psock,
                                     FAR struct sockaddr *addr,
                                     FAR socklen_t *addrlen,
                                     FAR struct socket *newsock)
{
  int                            ret;
  FAR struct devspecsock_conn_s *conn;
  FAR struct devspecsock_conn_s *newconn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_accept != NULL);

  sockif = conn->lower_sockif->sockif;

  newconn = devspecsock_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      ret = -ENOMEM;
      goto errout;
    }

  newsock->s_type = SOCK_DEVSPECSOCK_TYPE;
  newsock->s_conn = newconn;

  ret = sockif->si_accept(psock, addr, addrlen, newsock);
  if (ret < 0)
    {
      newsock->s_type = PF_UNSPEC;
      newsock->s_conn = NULL;
      goto errout_free;
    }

  newsock->s_sockif     = psock->s_sockif;
  newconn->lower_sockif = conn->lower_sockif;
  newconn->crefs        = 1;

  return OK;

errout_free:
  devspecsock_free(newconn);

errout:
  return ret;
}

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Name: devspecsock_sockif_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int devspecsock_sockif_poll(FAR struct socket *psock,
                                   FAR struct pollfd *fds, bool setup)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_poll != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_poll(psock, fds, setup);
}

#endif

/****************************************************************************
 * Name: devspecsock_sockif_send
 *
 * Description:
 *   The send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t devspecsock_sockif_send(FAR struct socket *psock,
                                       FAR const void *buf,
                                       size_t len, int flags)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_send != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_send(psock, buf, len, flags);
}

/****************************************************************************
 * Name: devspecsock_sockif_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM,
 *   SOCK_SEQPACKET) socket, the parameters to and 'tolen' are ignored
 *   (and the error EISCONN may be returned when they are not NULL and 0),
 *   and the error ENOTCONN is returned when the socket was not actually
 *   connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 ****************************************************************************/

static ssize_t devspecsock_sockif_sendto(FAR struct socket *psock,
                                         FAR const void *buf, size_t len,
                                         int flags,
                                         FAR const struct sockaddr *to,
                                         socklen_t tolen)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_sendto != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_sendto(psock, buf, len, flags, to, tolen);
}


/****************************************************************************
 * Name: devspecsock_sockif_sendfile
 *
 * Description:
 *   The sendfile() call may be used only when the INET socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See sendfile() for a list
 *   appropriate error return values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SENDFILE
static ssize_t devspecsock_sockif_sendfile(FAR struct socket *psock,
                                           FAR struct file *infile,
                                           FAR off_t *offset, size_t count)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_sendfile != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_sendfile(psock, infile, offset, count);
}
#endif

/****************************************************************************
 * Name: devspecsock_sockif_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags (ignored)
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 ****************************************************************************/

static ssize_t devspecsock_sockif_recvfrom(FAR struct socket *psock,
                                           FAR void *buf, size_t len,
                                           int flags,
                                           FAR struct sockaddr *from,
                                           FAR socklen_t *fromlen)
{
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_recvfrom != NULL);

  sockif = conn->lower_sockif->sockif;

  return sockif->si_recvfrom(psock, buf, len, flags, from, fromlen);
}

/****************************************************************************
 * Name: devspecsock_sockif_close
 *
 * Description:
 *   Performs the close operation on an device-specific socket instance
 *
 * Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int devspecsock_sockif_close(FAR struct socket *psock)
{
  int                            ret;
  FAR struct devspecsock_conn_s *conn;
  FAR const struct sock_intf_s  *sockif;


  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;

  DEBUGASSERT(conn->lower_sockif != NULL &&
              conn->lower_sockif->sockif != NULL &&
              conn->lower_sockif->sockif->si_close != NULL);

  sockif = conn->lower_sockif->sockif;

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform device-specific socket close. */

      ret = sockif->si_close(psock);

      /* Free the connection structure */

      conn->crefs = 0;
      devspecsock_free(conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: usrsock_close failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
