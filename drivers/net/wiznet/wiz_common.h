/****************************************************************************
 * drivers/net/wiznet/wiz_common.h
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

#ifndef __NET_WIZNET_WIZ_COMMON_H__
#define __NET_WIZNET_WIZ_COMMON_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <netdb.h>
#include <pthread.h>
#include <poll.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/wiznet.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_WIZNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WIZNET_INITIALIZE_WAIT     (30)      /* Wait for SPI setup done */
#define WIZNET_SET_NET_WAIT_COUNT  (256)     /* Wait for setup (US*COUNT) */
#define WIZNET_SET_NET_WAIT_US     (100000)  /* Wait for setup (US*COUNT) */
#define WIZNET_ACCEPT_WAIT_US      (500000)  /* Wait for accept */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* WIZnet IP information */

typedef struct
{
  uint8_t   mac[6];    /* MAC address */
  bool      dhcp;      /* DHCP configuration */
  in_addr_t ip;        /* IP address */
  in_addr_t mask;      /* Netmask */
  in_addr_t gw;        /* Gateway address */
  in_addr_t dns;       /* DNS server address */
} wiznet_ipaddr_t;

struct wiznet_dev_s
{
  FAR char                        *path;
  FAR struct pollfd               *pfd;
  FAR struct spi_dev_s            *spi;
  struct net_driver_s             net_dev;
  FAR const struct wiznet_lower_s *lower;

  uint8_t                         *buffer;
  sem_t                           lock_sem;

  uint8_t                         dns_server[4];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wiznet_lock_access()
 ****************************************************************************/

void wiznet_lock_access(FAR struct wiznet_dev_s *dev);

/****************************************************************************
 * Name: wiznet_unlock_access()
 ****************************************************************************/

void wiznet_unlock_access(FAR struct wiznet_dev_s *dev);

/****************************************************************************
 * Name: wiznet_initialize()
 ****************************************************************************/

int wiznet_initialize(FAR struct wiznet_dev_s *priv);

/****************************************************************************
 * Name: wiznet_finalize()
 ****************************************************************************/

int wiznet_finalize(FAR struct wiznet_dev_s *dev);

/****************************************************************************
 * Name: wiznet_setup()
 ****************************************************************************/

int wiznet_setup(FAR struct wiznet_dev_s *dev,
                 FAR struct wiznet_device_msg *msg);

/****************************************************************************
 * Name: wiznet_convert_error()
 ****************************************************************************/

int wiznet_convert_error(int value);

/****************************************************************************
 * Name: wiznet_get_net
 ****************************************************************************/

int wiznet_get_net(FAR struct wiznet_dev_s *dev, wiznet_ipaddr_t *ip);

/****************************************************************************
 * Name: wiznet_set_net
 ****************************************************************************/

int wiznet_set_net(FAR struct wiznet_dev_s *dev, wiznet_ipaddr_t *ip);

/****************************************************************************
 * Name: wiznet_poll
 ****************************************************************************/

int wiznet_poll(FAR struct wiznet_dev_s *dev);

/****************************************************************************
 * Name: wiznet_bind()
 ****************************************************************************/

int wiznet_bind(int sockfd,
                FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: wiznet_accept()
 ****************************************************************************/

int wiznet_accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);

/****************************************************************************
 * Name:  wiznet_check_interrupt()
 ****************************************************************************/

int wiznet_check_interrupt(FAR struct wiznet_dev_s *dev);

/****************************************************************************
 * Name:  wiznet_reset_interrupt()
 ****************************************************************************/

void wiznet_reset_interrupt(FAR struct wiznet_dev_s *dev, int sockfd);

#endif /* CONFIG_NET_WIZNET */
#endif  /*  __NET_WIZNET_WIZ_COMMON_H__ */
