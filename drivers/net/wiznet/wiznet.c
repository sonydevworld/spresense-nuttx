/****************************************************************************
 * drivers/net/wiznet/wiznet.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/net/wiznet.h>

#include "wiz_socket.h"
#include "wiz_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#define SPI_MAXFREQUENCY   CONFIG_NET_WIZNET_SPI_FREQUENCY
#define SPI_USESPIMODE     (SPIDEV_MODE3)

/* Avoid mixed case identifier warning */

#define SN_MR_TCP      Sn_MR_TCP
#define SN_MR_UDP      Sn_MR_UDP
#define SN_MR_MACRAW   Sn_MR_MACRAW

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int     wizdev_open(FAR struct file *filep);
static int     wizdev_close(FAR struct file *filep);
static ssize_t wizdev_read(FAR struct file *filep, FAR char *buff,
                           size_t len);
static ssize_t wizdev_write(FAR struct file *filep, FAR const char *buff,
                            size_t len);
static int     wizdev_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     wizdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/* Interrupt handler */

static int     wizdev_irq(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_wiznet_fops =
{
  wizdev_open,  /* open */
  wizdev_close, /* close */
  wizdev_read,  /* read */
  wizdev_write, /* write */
  NULL,         /* seek */
  wizdev_ioctl, /* ioctl */
  wizdev_poll,  /* poll */
  NULL          /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wizdev_open
 ****************************************************************************/

static int wizdev_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: wizdev_close
 ****************************************************************************/

static int wizdev_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: wizdev_read
 ****************************************************************************/

static ssize_t wizdev_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode;
  FAR struct wiznet_dev_s *dev;
  int  ret = 0;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct wiznet_dev_s *)inode->i_private;

  wiznet_lock_access(dev);

  ret = wiznet_check_interrupt(dev);
  if (ret <= 0)
    {
      ret = 0;
    }

  memcpy(buffer, &ret, sizeof(ret));
  ret = sizeof(ret);

  wiznet_unlock_access(dev);

  return ret;
}

/****************************************************************************
 * Name: wizdev_write
 ****************************************************************************/

static ssize_t wizdev_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len)
{
  return 0;
}

/****************************************************************************
 * Name: wiznet_spi_init
 ****************************************************************************/

static int wiznet_spi_init(FAR struct wiznet_dev_s *dev)
{
  (void)SPI_LOCK(dev->spi, true);

  SPI_SETMODE(dev->spi, SPI_USESPIMODE);
  SPI_SETBITS(dev->spi, 8);
  (void)SPI_SETFREQUENCY(dev->spi, SPI_MAXFREQUENCY);

  (void)SPI_LOCK(dev->spi, false);

  dev->lower->poweron(true);

  /* Attach interrupt handler */

  dev->lower->attach(true, wizdev_irq, dev);

  return 0;
}

/****************************************************************************
 * Name: wiznet_ioctl_device
 ****************************************************************************/

static int wiznet_ioctl_device(FAR struct wiznet_dev_s *dev,
                               FAR struct wiznet_device_msg *msg)
{
  return wiznet_setup(dev, msg);
}

/****************************************************************************
 * Name: wiznet_ioctl_socket
 ****************************************************************************/

static int wiznet_ioctl_socket(FAR struct wiznet_dev_s *dev,
                               FAR struct wiznet_socket_msg *msg)
{
  int ret = OK;
  int type = SOCK_STREAM;

  switch (msg->type)
    {
      case SOCK_STREAM:
        type = SN_MR_TCP;
        break;
      case SOCK_DGRAM:
        type = SN_MR_UDP;
        break;
      case SOCK_RAW:
        type = SN_MR_MACRAW;
        break;
      default:
        break;
    }

  ret = wiz_socket(msg->sockfd, type, 0, 0);

  if (SOCK_OK == ret)
    {
      ret = OK;
    }
  else
    {
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_connect
 ****************************************************************************/

static int wiznet_ioctl_connect(FAR struct wiznet_dev_s *dev,
                                FAR struct wiznet_connect_msg *msg)
{
  int ret = OK;
  struct sockaddr_in *addr = (struct sockaddr_in *)&msg->addr;

  ret = wiz_connect(msg->sockfd,
                    (uint8_t *)&(addr->sin_addr.s_addr),
                    ntohs(addr->sin_port));

  if (SOCK_OK == ret)
    {
      ret = OK;
    }
  else
    {
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_send
 ****************************************************************************/

static int wiznet_ioctl_send(FAR struct wiznet_dev_s *dev,
                             FAR struct wiznet_send_msg *msg)
{
  int ret = OK;
  struct sockaddr_in *addr = (struct sockaddr_in *)&msg->addr;

  do
    {
      if (SOCK_STREAM == msg->type)
        {
          ret = wiz_send(msg->sockfd, (void *)msg->buf, msg->len);
        }
      else
        {
          ret = wiz_sendto(msg->sockfd, (void *)msg->buf, msg->len,
                           (uint8_t *)&(addr->sin_addr.s_addr),
                           ntohs(addr->sin_port));
        }
    }
  while (SOCK_BUSY == ret);

  if (SOCK_OK <= ret)
    {
      msg->result = ret;
      ret = OK;
    }
  else
    {
      msg->result = 0;
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_recv
 ****************************************************************************/

static int wiznet_ioctl_recv(FAR struct wiznet_dev_s *dev,
                             FAR struct wiznet_recv_msg *msg)
{
  int ret  = OK;
  struct sockaddr_in *addr = (struct sockaddr_in *)&msg->addr;

  do
    {
      if (SOCK_STREAM == msg->type)
        {
          ret = wiz_recv(msg->sockfd, (void *)msg->buf, msg->len);
        }
      else
        {
          ret = wiz_recvfrom(msg->sockfd, (void *)msg->buf, msg->len,
                             (uint8_t *)&(addr->sin_addr.s_addr),
                             &(addr->sin_port));
        }
    }
  while (SOCK_BUSY == ret);

  if (SOCK_OK <= ret)
    {
      msg->result = ret;
      ret = OK;

      wiznet_reset_interrupt(dev, msg->sockfd);
    }
  else
    {
      msg->result = 0;
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_close
 ****************************************************************************/

static int wiznet_ioctl_close(FAR struct wiznet_dev_s *dev,
                              FAR struct wiznet_close_msg *msg)
{
  int ret = OK;
  uint8_t val;

  wiz_getsockopt(msg->sockfd, SO_STATUS, &val);

  if (SOCK_ESTABLISHED == val)
    {
      wiz_disconnect(msg->sockfd);
    }

  ret = wiz_close(msg->sockfd);

  if (SOCK_OK == ret)
    {
      ret = 0;
    }
  else
    {
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_bind
 ****************************************************************************/

static int wiznet_ioctl_bind(FAR struct wiznet_dev_s *dev,
                             FAR struct wiznet_bind_msg *msg)
{
  int ret = OK;

  ret = wiznet_bind(msg->sockfd,
                    (struct sockaddr *)&(msg->addr), msg->addrlen);

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_listen
 ****************************************************************************/

static int wiznet_ioctl_listen(FAR struct wiznet_dev_s *dev,
                               FAR struct wiznet_listen_msg *msg)
{
  int ret = OK;

  ret = wiz_listen(msg->sockfd);

  if (SOCK_OK == ret)
    {
      ret = OK;
    }
  else
    {
      ret = wiznet_convert_error(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_accept
 ****************************************************************************/

static int wiznet_ioctl_accept(FAR struct wiznet_dev_s *dev,
                               FAR struct wiznet_accept_msg *msg)
{
  int ret = OK;

  ret = wiznet_accept(msg->sockfd,
                      (struct sockaddr *)&(msg->addr), &(msg->addrlen));

  return ret;
}

/****************************************************************************
 * Name: wiznet_ifreq_ifreq
 ****************************************************************************/

static int wiznet_ioctl_ifreq(FAR struct wiznet_dev_s *dev,
                              FAR struct wiznet_ifreq_msg *msg)
{
  wiznet_ipaddr_t netinfo;
  struct sockaddr_in *addr;
  int ret = OK;

  switch (msg->cmd)
    {
      case SIOCGIFHWADDR:
      case SIOCGIFADDR:
      case SIOCGIFBRDADDR:
      case SIOCGIFNETMASK:
      case SIOCSIFHWADDR:
      case SIOCSIFADDR:
      case SIOCSIFBRDADDR:
      case SIOCSIFNETMASK:
        wiznet_get_net(dev, &netinfo);
        break;

      default:
        ret = SOCKERR_SOCKFLAG;
        break;
    }

  switch (msg->cmd)
    {
      case SIOCGIFHWADDR:
        memmove((uint8_t *)&msg->ifr.ifr_hwaddr.sa_data,
                (uint8_t *)&netinfo.mac, 6);
        break;

      case SIOCGIFADDR:
        addr = (struct sockaddr_in *)&msg->ifr.ifr_addr;
        memmove((uint8_t *)&addr->sin_addr.s_addr,
                (uint8_t *)&netinfo.ip, 4);
        break;

      case SIOCGIFBRDADDR:
        addr = (struct sockaddr_in *)&msg->ifr.ifr_broadaddr;
        memmove((uint8_t *)&addr->sin_addr.s_addr,
                (uint8_t *)&netinfo.gw, 4);
        break;

      case SIOCGIFNETMASK:
        addr = (struct sockaddr_in *)&msg->ifr.ifr_netmask;
        memmove((uint8_t *)&addr->sin_addr.s_addr,
                (uint8_t *)&netinfo.mask, 4);
        break;

      case SIOCSIFHWADDR:
        memmove((uint8_t *)&netinfo.mac,
                (uint8_t *)&msg->ifr.ifr_hwaddr.sa_data, 6);
        wiznet_set_net(dev, &netinfo);
        break;

      case SIOCSIFADDR:
        netinfo.dhcp = false;
        addr = (struct sockaddr_in *)&msg->ifr.ifr_addr;
        memmove((uint8_t *)&netinfo.ip,
                (uint8_t *)&addr->sin_addr.s_addr, 4);
        wiznet_set_net(dev, &netinfo);
        break;

      case SIOCSIFBRDADDR:
        netinfo.dhcp = false;
        addr = (struct sockaddr_in *)&msg->ifr.ifr_broadaddr;
        memmove((uint8_t *)&netinfo.gw,
                (uint8_t *)&addr->sin_addr.s_addr, 4);
        wiznet_set_net(dev, &netinfo);
        break;

      case SIOCSIFNETMASK:
        netinfo.dhcp = false;
        addr = (struct sockaddr_in *)&msg->ifr.ifr_netmask;
        memmove((uint8_t *)&netinfo.mask,
                (uint8_t *)&addr->sin_addr.s_addr, 4);
        wiznet_set_net(dev, &netinfo);
        break;

      default:
        ret = SOCKERR_SOCKFLAG;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: wiznet_ioctl_name
 ****************************************************************************/

static int wiznet_ioctl_name(FAR struct wiznet_dev_s *dev,
                             FAR struct wiznet_name_msg *msg)
{
  int ret = 0;
  wiznet_ipaddr_t netinfo;
  struct sockaddr_in *addr = (struct sockaddr_in *)&msg->addr;

  memset(addr, 0, sizeof(struct sockaddr_in));

  if (msg->local)
    {
      wiznet_get_net(dev, &netinfo);
      memmove((uint8_t *)&(addr->sin_addr.s_addr),
              (uint8_t *)&netinfo.ip, sizeof(in_addr_t));
    }
  else
    {
      ret = wiz_getsockopt(msg->sockfd, SO_DESTIP, &netinfo.ip);

      if (SOCK_OK == ret)
        {
          memmove((uint8_t *)&(addr->sin_addr.s_addr),
                  (uint8_t *)&netinfo.ip, sizeof(in_addr_t));
        }
      else
        {
          ret = wiznet_convert_error(ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: wizdev_ioctl
 ****************************************************************************/

static int wizdev_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct wiznet_dev_s *dev;
  int ret = -EINVAL;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct wiznet_dev_s *)inode->i_private;

  /* Lock the device */

  ninfo("== ioctl:%X\n", cmd);
  wiznet_lock_access(dev);

  /* Disable wiznet irq to poll dready */

  DEBUGASSERT(dev);

  switch (cmd)
    {
      case WIZNET_IOC_DEVICE:
        {
          struct wiznet_device_msg *msg =
            (struct wiznet_device_msg *)arg;

          ret = wiznet_ioctl_device(dev, msg);
        }
        break;

      case WIZNET_IOC_SOCKET:
        {
          struct wiznet_socket_msg *msg =
            (struct wiznet_socket_msg *)arg;

          ret = wiznet_ioctl_socket(dev, msg);
        }
        break;

      case WIZNET_IOC_CONNECT:
        {
          struct wiznet_connect_msg *msg =
            (struct wiznet_connect_msg *)arg;

          ret = wiznet_ioctl_connect(dev, msg);
        }
        break;

      case WIZNET_IOC_SEND:
        {
          struct wiznet_send_msg *msg =
            (struct wiznet_send_msg *)arg;

          ret = wiznet_ioctl_send(dev, msg);
        }
        break;

      case WIZNET_IOC_RECV:
        {
          struct wiznet_recv_msg *msg =
            (struct wiznet_recv_msg *)arg;

          ret = wiznet_ioctl_recv(dev, msg);
          break;
        }

      case WIZNET_IOC_CLOSE:
        {
          struct wiznet_close_msg *msg =
            (struct wiznet_close_msg *)arg;

          ret = wiznet_ioctl_close(dev, msg);
          break;
        }

      case WIZNET_IOC_BIND:
        {
          struct wiznet_bind_msg *msg =
            (struct wiznet_bind_msg *)arg;

          ret = wiznet_ioctl_bind(dev, msg);
          break;
        }

      case WIZNET_IOC_LISTEN:
        {
          struct wiznet_listen_msg *msg =
            (struct wiznet_listen_msg *)arg;

          ret = wiznet_ioctl_listen(dev, msg);
          break;
        }

      case WIZNET_IOC_ACCEPT:
        {
          struct wiznet_accept_msg *msg =
            (struct wiznet_accept_msg *)arg;

          ret = wiznet_ioctl_accept(dev, msg);
          break;
        }

      case WIZNET_IOC_IFREQ:
        {
          struct wiznet_ifreq_msg *msg =
            (struct wiznet_ifreq_msg *)arg;

          ret = wiznet_ioctl_ifreq(dev, msg);
          break;
        }

      case WIZNET_IOC_NAME:
        {
          struct wiznet_name_msg *msg =
            (struct wiznet_name_msg *)arg;

          ret = wiznet_ioctl_name(dev, msg);
          break;
        }

      default:
        DEBUGPANIC();
        break;
    }

  /* Unlock the device */

  wiznet_unlock_access(dev);
  ninfo("== ioctl:%X finished\n", cmd);

  return ret;
}

/****************************************************************************
 * Name: wizdev_poll
 ****************************************************************************/

static int wizdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode;
  FAR struct wiznet_dev_s *dev;
  int ret = OK;

  ninfo("== setup:%d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct wiznet_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* NOTE: only one thread can poll the device at any time */

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;
      dev->lower->enable(true);
    }
  else
    {
      dev->lower->enable(false);
      dev->pfd = NULL;
    }

errout:
  ninfo("== setup:%d finished\n", (int)setup);

  return ret;
}

/****************************************************************************
 * Name: wiznet_interrupt
 ****************************************************************************/

static int wizdev_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct wiznet_dev_s *dev;

  DEBUGASSERT(arg != NULL);
  dev = (FAR struct wiznet_dev_s *)arg;

  if (dev->pfd)
    {
      ninfo("== interrupted : post\n");
      dev->pfd->revents |= POLLIN;
      nxsem_post(dev->pfd->sem);
    }

  dev->lower->enable(false);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wiznet_register
 ****************************************************************************/

FAR void * wiznet_register(FAR const char *devpath,
                           FAR struct spi_dev_s *spi,
                           FAR const struct wiznet_lower_s *lower)
{
  struct wiznet_dev_s *priv;
  int ret;
  int size = sizeof(struct wiznet_dev_s);

  priv = (FAR struct wiznet_dev_s *)kmm_malloc(size);
  if (!priv)
    {
      nerr("Failed to allocate instance.\n");
      return NULL;
    }

  memset(priv, 0, size);

  priv->spi   = spi;
  priv->path  = strdup(devpath);
  priv->lower = lower;
  priv->pfd   = NULL;
  nxsem_init(&priv->lock_sem, 0, 1);

  ret = wiznet_spi_init(priv);

  if (ret < 0)
    {
      nerr("Failed to initialize WIZNET driver.\n");
      kmm_free(priv);
      return NULL;
    }

  ret = wiznet_initialize(priv);

  if (ret < 0)
    {
      nerr("Failed to initialize WIZNET driver.\n");
      kmm_free(priv);
      return NULL;
    }

  ret = register_driver(devpath, &g_wiznet_fops, 0666, priv);

  if (ret < 0)
    {
      nerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  ret = netdev_register(&priv->net_dev, NET_LL_ETHERNET);

  ninfo("Register WIZNET driver.\n");
  return (FAR void *)priv;
}

