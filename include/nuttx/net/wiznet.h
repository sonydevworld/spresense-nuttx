/****************************************************************************
 * include/nuttx/net/wiznet.h
 *
 *   Copyright 2020 Sony Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __INCLUDE_NUTTX_NET_WIZNET_H
#define __INCLUDE_NUTTX_NET_WIZNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/net/ioctl.h>

/****************************************************************************
 * Public Definitions
 ****************************************************************************/

#define WIZNET_IOC_DEVICE  _SIOC(0x002a)
#define WIZNET_IOC_SOCKET  _SIOC(0x002b)
#define WIZNET_IOC_CONNECT _SIOC(0x002c)
#define WIZNET_IOC_SEND    _SIOC(0x002d)
#define WIZNET_IOC_RECV    _SIOC(0x002e)
#define WIZNET_IOC_CLOSE   _SIOC(0x002f)
#define WIZNET_IOC_BIND    _SIOC(0x0030)
#define WIZNET_IOC_LISTEN  _SIOC(0x0031)
#define WIZNET_IOC_ACCEPT  _SIOC(0x0032)
#define WIZNET_IOC_IFREQ   _SIOC(0x0033)
#define WIZNET_IOC_NAME    _SIOC(0x0034)

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* NOTE: do not forget to update include/nuttx/net/ioctl.h */

struct wiznet_device_msg
{
  bool            dhcp;
  in_addr_t       ipaddr;
  in_addr_t       draddr;
  in_addr_t       netmask;
  in_addr_t       dnsaddr;
  uint64_t        mac;
};

struct wiznet_socket_msg
{
  int             sockfd;
  int             domain;
  int             type;
  int             protocol;
};

struct wiznet_connect_msg
{
  int             sockfd;
  int             type;
  struct sockaddr addr;
  socklen_t       addrlen;
};

struct wiznet_bind_msg
{
  int             sockfd;
  int             type;
  struct sockaddr addr;
  socklen_t       addrlen;
};

struct wiznet_listen_msg
{
  int             sockfd;
  int             type;
  int             backlog;
};

struct wiznet_accept_msg
{
  int             sockfd;
  int             type;
  struct sockaddr addr;
  socklen_t       addrlen;
};

struct wiznet_send_msg
{
  int             sockfd;
  int             type;
  FAR uint8_t     *buf;
  size_t          len;
  int             flags;
  struct sockaddr addr;
  socklen_t       addrlen;
  ssize_t         result;
};

struct wiznet_recv_msg
{
  int             sockfd;
  int             type;
  FAR uint8_t     *buf;
  size_t          len;
  int             flags;
  struct sockaddr addr;
  socklen_t       addrlen;
  ssize_t         result;
};

struct wiznet_close_msg
{
  int             sockfd;
  int             type;
};

struct wiznet_ifreq_msg
{
  uint32_t        cmd;
  struct ifreq    ifr;
};

struct wiznet_name_msg
{
  int             sockfd;
  struct sockaddr addr;
  socklen_t       addrlen;
  bool            local;
};

struct wiznet_lower_s
{
  void (*attach)(bool attach, xcpt_t handler, FAR void *arg);
  void (*enable)(bool enable);
  void (*poweron)(bool on);
};

FAR void * wiznet_register(FAR const char *devpath,
                           FAR struct spi_dev_s *spi,
                           FAR const struct wiznet_lower_s *lower);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_NET_WIZNET_H */
