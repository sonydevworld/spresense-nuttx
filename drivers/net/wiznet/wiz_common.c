/****************************************************************************
 * drivers/net/wiznet/wiz_common.c
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

#include "stdio.h"
#include "stdlib.h"
#include "errno.h"
#include "fcntl.h"
#include "string.h"
#include <netdb.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <signal.h>
#include <nuttx/net/dns.h>

#include "wiz_common.h"
#include "wiz_socket.h"
#include "wizchip_conf.h"
#include "wiz_dhcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WIZNET_INTERNET_SOCKET      0
#define WIZNET_DHCP_BUFFER_SIZE     548

#define WIZNET_REG_RETRY_COUNT      8
#define WIZNET_REG_RETRY_TIMEOUT    2000

/* Avoid mixed case identifier warning */

#define wiz_netinfo       wiz_NetInfo
#define wiz_nettimeout    wiz_NetTimeout

#define dhcp_time_handler DHCP_time_handler
#define dhcp_init         DHCP_init
#define dhcp_run          DHCP_run
#define dhcp_stop         DHCP_stop
#define get_dns_from_dhcp getDNSfromDHCP

#define get_sn_sr         getSn_SR
#define get_sn_ir         getSn_IR
#define set_sn_ir         setSn_IR
#define get_sn_rx_rsr     getSn_RX_RSR
#define set_sn_port       setSn_PORT

#define SN_IR_CHECK       (Sn_IR_RECV|Sn_IR_DISCON|Sn_IR_CON)
#define SN_IR_RESET       (Sn_IR_RECV|Sn_IR_CON)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct wiznet_dev_s      *g_wizdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wiznet_cris_enter
 ****************************************************************************/

static void wiznet_cris_enter(void)
{
}

/****************************************************************************
 * Name: wiznet_cris_exit
 ****************************************************************************/

static void wiznet_cris_exit(void)
{
}

/****************************************************************************
 * Name: wiznet_cs_select
 ****************************************************************************/

static void wiznet_cs_select(void)
{
  /* Low active signal */

  SPI_SELECT(g_wizdev->spi, 0, false);
}

/****************************************************************************
 * Name: wiznet_cs_deselect
 ****************************************************************************/

static void wiznet_cs_deselect(void)
{
  /* Low active signal */

  SPI_SELECT(g_wizdev->spi, 0, true);
}

/****************************************************************************
 * Name: wiznet_spi_readbyte
 ****************************************************************************/

static uint8_t wiznet_spi_readbyte(void)
{
  uint8_t rb = 0;

  if (g_wizdev != NULL)
    {
      SPI_EXCHANGE(g_wizdev->spi, &rb, &rb, 1);
    }

  return rb;
}

/****************************************************************************
 * Name: wiznet_spi_writebyte
 ****************************************************************************/

static void wiznet_spi_writebyte(uint8_t wb)
{
  if (g_wizdev != NULL)
    {
      SPI_EXCHANGE(g_wizdev->spi, &wb, NULL, 1);
    }
}

/****************************************************************************
 * Name: wiznet_spi_readburst
 ****************************************************************************/

static void wiznet_spi_readburst(uint8_t *pbuf, uint16_t len)
{
  if (g_wizdev != NULL)
    {
      SPI_EXCHANGE(g_wizdev->spi, pbuf, pbuf, len);
    }
}

/****************************************************************************
 * Name: wiznet_spi_writeburst
 ****************************************************************************/

static void wiznet_spi_writeburst(uint8_t *pbuf, uint16_t len)
{
  if (g_wizdev != NULL)
    {
      SPI_EXCHANGE(g_wizdev->spi, pbuf, NULL, len);
    }
}

/****************************************************************************
 * Name: wiznet_start_timer
 ****************************************************************************/

static timer_t wiznet_start_timer(int interval_ms, FAR void *handler)
{
  int ret;
  sigset_t mask;
  struct sigaction sa;
  struct sigevent sev;
  struct itimerspec timer;
  timer_t timerid;

  sigemptyset(&mask);
  sigaddset(&mask, SIGUSR1);

  ret = sigprocmask(SIG_UNBLOCK, &mask, NULL);
  if (ret != OK)
    {
      nerr("sigprocmask() failed:%d\n", ret);
      return NULL;
    }

  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO;
  sigfillset(&sa.sa_mask);
  sigdelset(&sa.sa_mask, SIGUSR1);

  ret = sigaction(SIGUSR1, &sa, NULL);
  if (ret != OK)
    {
      nerr("sigaction() failed:%d\n", ret);
      return NULL;
    }

  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIGUSR1;

  ret = timer_create(CLOCK_REALTIME, &sev, &timerid);
  if (ret != OK)
    {
      nerr("timer_create() failed:%d\n", ret);
      return NULL;
    }

  timer.it_value.tv_sec = interval_ms / 1000;
  timer.it_value.tv_nsec = (interval_ms % 1000) * 1000 * 1000;
  timer.it_interval.tv_sec  = timer.it_value.tv_sec;
  timer.it_interval.tv_nsec = timer.it_value.tv_nsec;

  ret = timer_settime(timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      nerr("timer_settime() failed:%d\n", ret);
      return NULL;
    }

  return timerid;
}

/****************************************************************************
 * Name: wiznet_stop_timer
 ****************************************************************************/

static void wiznet_stop_timer(timer_t timerid)
{
  sigset_t mask;

  timer_delete(timerid);

  sigfillset(&mask);
  sigprocmask(SIG_SETMASK, &mask, NULL);
}

/****************************************************************************
 * Name: wiznet_internet_dhcp
 ****************************************************************************/

static int wiznet_internet_dhcp(FAR struct wiznet_dev_s *dev)
{
  int ret;
  timer_t timerid;
  uint8_t dhcp_buffer[WIZNET_DHCP_BUFFER_SIZE];

  ninfo("wiznet_internet_dhcp start\n");

  timerid = wiznet_start_timer(1000, dhcp_time_handler);

  dhcp_init(WIZNET_INTERNET_SOCKET, dhcp_buffer);

  do
    {
      ret = dhcp_run();
      nxsig_sleep(1);
    }
  while (ret == DHCP_RUNNING);

  dhcp_stop();

  wiznet_stop_timer(timerid);

  if ((ret == DHCP_FAILED) || (ret == DHCP_STOPPED))
    {
      nerr("wiznet_internet_dhcp failed : %d\n", ret);
      errno = EHOSTDOWN;
      return -1;
    }

  ninfo("wiznet_internet_dhcp success : %d\n", ret);
  return 0;
}

/****************************************************************************
 * Name: wiznet_check_netinfo
 ****************************************************************************/

static int wiznet_check_netinfo(wiz_netinfo *netinfo)
{
  int ret = -1;
  int cnt;
  wiz_netinfo netcheck;

  for (cnt = 0; cnt < WIZNET_SET_NET_WAIT_COUNT; cnt++)
    {
      wizchip_setnetinfo(netinfo);
      wizchip_getnetinfo(&netcheck);

      if (memcmp(netinfo, &netcheck, 6) == 0)
        {
          ret = 0;
          break;
        }

      /* Wait before next try */

      nxsig_usleep(WIZNET_SET_NET_WAIT_US);
    }

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wiznet_initialize
 *
 * Description:
 *   initialise WIZNET class.
 *      - open port
 *      - configure port
 *      - etc...
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *
 * Returned Value:
 *   0 on success, -1 in case of error.
 *
 ****************************************************************************/

int wiznet_initialize(FAR struct wiznet_dev_s *dev)
{
  int cnt;
  netmode_type netmode;
  wiz_nettimeout nettime;
  uint8_t bufsize[_WIZCHIP_SOCK_NUM_];

  ninfo("Initializing WIZNET...\n");

  /* Register private device */

  g_wizdev = dev;

#if defined(CONFIG_NET_WIZNET_W5500)
  /* Buffer allocation */

  switch (_WIZCHIP_SOCK_NUM_)
    {
      case 1:
          dev->buffer = WIZCHIP_BUFFER(16384);
          break;
      case 2:
          dev->buffer = WIZCHIP_BUFFER(8192);
          break;
      case 4:
          dev->buffer = WIZCHIP_BUFFER(4096);
          break;
      default:
          dev->buffer = WIZCHIP_BUFFER(2048);
          break;
    }

  if (dev->buffer == NULL)
    {
      nerr("WIZNET alloc memory failed\n");
      errno = ENOMEM;
      return -1;
    }
#endif

  /* Initialize */

  reg_wizchip_cris_cbfunc(wiznet_cris_enter, wiznet_cris_exit);
  reg_wizchip_cs_cbfunc(wiznet_cs_select, wiznet_cs_deselect);
  reg_wizchip_spi_cbfunc(wiznet_spi_readbyte, wiznet_spi_writebyte);
  reg_wizchip_spiburst_cbfunc(wiznet_spi_readburst, wiznet_spi_writeburst);

  ninfo("WIZNET waiting for initialize\n");

  for (cnt = 0; cnt < WIZNET_INITIALIZE_WAIT; cnt++)
    {
      netmode = wizchip_getnetmode();
      if (!(netmode & 0x80))
        {
          ninfo("WIZNET waiting skipped\n");
          break;
        }

      /* Wait for transmission */

      nxsig_sleep(1);
    }

  for (cnt = 0; cnt < WIZNET_INITIALIZE_WAIT; cnt++)
    {
      /* Set default timeout */

      nettime.retry_cnt = WIZNET_REG_RETRY_COUNT;
      nettime.time_100us = WIZNET_REG_RETRY_TIMEOUT;
      wizchip_settimeout(&nettime);
      nxsig_sleep(1);
      wizchip_gettimeout(&nettime);

      if ((nettime.retry_cnt == WIZNET_REG_RETRY_COUNT)
          && (nettime.time_100us == WIZNET_REG_RETRY_TIMEOUT))
        {
          ninfo("WIZNET waiting skipped\n");
          break;
        }
    }

  switch (_WIZCHIP_SOCK_NUM_)
    {
      case 1:
          memset(bufsize, 16, _WIZCHIP_SOCK_NUM_);
          break;
      case 2:
          memset(bufsize, 8, _WIZCHIP_SOCK_NUM_);
          break;
      case 4:
          memset(bufsize, 4, _WIZCHIP_SOCK_NUM_);
          break;
      default:
          memset(bufsize, 2, _WIZCHIP_SOCK_NUM_);
          break;
    }

  wizchip_init(bufsize, bufsize);

  /* Setup IRQ */

  wizchip_setinterruptmask(IK_SOCK_ALL);
  wizchip_clrinterrupt(IK_SOCK_ALL);

  ninfo("WIZNET initialized\n");
  memset(dev->dns_server, 0, 4);

  errno = 0;

  return 0;
}

/****************************************************************************
 * Name: wiznet_finalize
 *
 * Description:
 *   finalize WIZNET class.
 *      - close port
 *      - etc...
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *
 * Returned Value:
 *   0 on success, -1 in case of error.
 *
 ****************************************************************************/

int wiznet_finalize(FAR struct wiznet_dev_s *dev)
{
  ninfo("Finalizing WIZNET...\n");

#if defined(CONFIG_NET_WIZNET_W5100S) || defined(CONFIG_NET_WIZNET_W5500)
  wizphy_reset();
#endif

#if defined(CONFIG_NET_WIZNET_W5500)
  dev->buffer = WIZCHIP_BUFFER(0);
#endif

  /* Unregister private device */

  g_wizdev = NULL;

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name: wiznet_setup
 *
 * Description:
 *   setup wiznet device
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *   msg  : device message
 *
 * Returned Value:
 *   0 on success, -1 in case of error.
 *
 ****************************************************************************/

int wiznet_setup(FAR struct wiznet_dev_s *dev,
                 FAR struct wiznet_device_msg *msg)
{
  int ret = 0;
  wiznet_ipaddr_t ip;

  /* MAC address configuration */

  ip.mac[0] = (msg->mac >> (8 * 5)) & 0xff;
  ip.mac[1] = (msg->mac >> (8 * 4)) & 0xff;
  ip.mac[2] = (msg->mac >> (8 * 3)) & 0xff;
  ip.mac[3] = (msg->mac >> (8 * 2)) & 0xff;
  ip.mac[4] = (msg->mac >> (8 * 1)) & 0xff;
  ip.mac[5] = (msg->mac >> (8 * 0)) & 0xff;

  /* IP configuration */

  ip.dhcp = msg->dhcp;
  if (!msg->dhcp)
    {
      memmove(&ip.ip, &msg->ipaddr, sizeof(in_addr_t));
      memmove(&ip.gw, &msg->draddr, sizeof(in_addr_t));
      memmove(&ip.mask, &msg->netmask, sizeof(in_addr_t));
      memmove(&ip.dns, &msg->dnsaddr, sizeof(in_addr_t));
    }

  /* Update configuration */

  ret = wiznet_set_net(dev, &ip);
  if (ret < 0)
    {
      return ret;
    }

  if (msg->dhcp)
    {
      memmove(&msg->ipaddr, &ip.ip, sizeof(in_addr_t));
      memmove(&msg->draddr, &ip.gw, sizeof(in_addr_t));
      memmove(&msg->netmask, &ip.mask, sizeof(in_addr_t));
      memmove(&msg->dnsaddr, &ip.dns, sizeof(in_addr_t));
    }

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name: wiznet_convert_error
 *
 * Description:
 *    confirm wiznet is connected
 *
 * Input Parmeters:
 *   value     : value to convert wiznet error code to errno
 *
 * Returned Value:
 *   0 or positive value for success case, negative vale for error case.
 *
 ****************************************************************************/

int wiznet_convert_error(int value)
{
  switch (value)
    {
      case SOCK_BUSY:
        errno = EAGAIN;
        break;
      case SOCKERR_SOCKNUM:
        errno = EBADF;
        break;
      case SOCKERR_SOCKOPT:
      case SOCKERR_SOCKFLAG:
      case SOCKERR_ARG:
      case SOCKERR_PORTZERO:
      case SOCKERR_IPINVALID:
        errno = EINVAL;
        break;
      case SOCKERR_SOCKINIT:
        errno = ENOTCONN;
        break;
      case SOCKERR_SOCKMODE:
        errno = EISCONN;
        break;
      case SOCKERR_SOCKCLOSED:
      case SOCKERR_SOCKSTATUS:
        errno = ECONNRESET;
        return 0;
      case SOCKERR_TIMEOUT:
        errno = ETIME;
        break;
      case SOCKERR_DATALEN:
      case SOCKERR_BUFFER:
      case SOCKFATAL_PACKLEN:
        errno = ENOMEM;
        break;
      default:
        errno = 0;
        return value;
    }

  return -errno;
}

/****************************************************************************
 * Name: wiznet_lock_access
 *
 * Description:
 *    Lock access right to wiznet
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void wiznet_lock_access(FAR struct wiznet_dev_s *dev)
{
  nxsem_wait_uninterruptible(&dev->lock_sem);
}

/****************************************************************************
 * Name: wiznet_unlock_access
 *
 * Description:
 *    Unlock access right to wiznet
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void wiznet_unlock_access(FAR struct wiznet_dev_s *dev)
{
  nxsem_post(&dev->lock_sem);
}

/****************************************************************************
 * Name: wiznet_set_net
 *
 * Description:
 *   It will set network ip of mode.
 *
 * Input Parmeters:
 *   dev  : private data for wiznet driver access
 *   ip   : ip address info structure of interface
 *
 * Returned Value:
 *   0 on success, -1 in case of error.
 *
 ****************************************************************************/

int wiznet_set_net(FAR struct wiznet_dev_s *dev, wiznet_ipaddr_t *ip)
{
  wiz_netinfo netinfo;
  struct sockaddr_in dnsaddr;

  /* Check parameter */

  if (ip == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  /* Reset internal parameter */

  dev->net_dev.d_flags |= IFF_DOWN;

  /* Create mac_addr data */

  memset(&netinfo, 0, sizeof(wiz_netinfo));
  memmove(netinfo.mac, ip->mac, 6);

  if (ip->dhcp)
    {
      netinfo.dhcp = NETINFO_DHCP;

      /* Setup network parameter */

      if (wiznet_check_netinfo(&netinfo) < 0)
        {
          errno = EIO;
          return -1;
        }

      /* Call DHCP function */

      if (wiznet_internet_dhcp(dev) < 0)
        {
          return -1;
        }

      /* Backup DNS server information */

      get_dns_from_dhcp(dev->dns_server);
    }
  else
    {
      netinfo.dhcp = NETINFO_STATIC;

      netinfo.ip[0] = *((uint8_t *)&(ip->ip)+0);
      netinfo.ip[1] = *((uint8_t *)&(ip->ip)+1);
      netinfo.ip[2] = *((uint8_t *)&(ip->ip)+2);
      netinfo.ip[3] = *((uint8_t *)&(ip->ip)+3);
      netinfo.sn[0] = *((uint8_t *)&(ip->mask)+0);
      netinfo.sn[1] = *((uint8_t *)&(ip->mask)+1);
      netinfo.sn[2] = *((uint8_t *)&(ip->mask)+2);
      netinfo.sn[3] = *((uint8_t *)&(ip->mask)+3);
      netinfo.gw[0] = *((uint8_t *)&(ip->gw)+0);
      netinfo.gw[1] = *((uint8_t *)&(ip->gw)+1);
      netinfo.gw[2] = *((uint8_t *)&(ip->gw)+2);
      netinfo.gw[3] = *((uint8_t *)&(ip->gw)+3);
      dev->dns_server[0] = *((uint8_t *)&(ip->dns)+0);
      dev->dns_server[1] = *((uint8_t *)&(ip->dns)+1);
      dev->dns_server[2] = *((uint8_t *)&(ip->dns)+2);
      dev->dns_server[3] = *((uint8_t *)&(ip->dns)+3);

      /* Setup network parameter */

      if (wiznet_check_netinfo(&netinfo) < 0)
        {
          errno = EIO;
          return -1;
        }
    }

  /* Update internal parameter */

  wiznet_get_net(dev, ip);

  dev->net_dev.d_flags |= IFF_UP;
  memmove(&dev->net_dev.d_mac.ether.ether_addr_octet, &ip->mac, 6);
  memmove(&dev->net_dev.d_ipaddr, &ip->ip, sizeof(in_addr_t));
  memmove(&dev->net_dev.d_draddr, &ip->gw, sizeof(in_addr_t));
  memmove(&dev->net_dev.d_netmask, &ip->mask, sizeof(in_addr_t));

  dnsaddr.sin_family = AF_INET;
  memmove(&dnsaddr.sin_addr, &ip->dns, sizeof(in_addr_t));
  dnsaddr.sin_port = htons(DNS_DEFAULT_PORT);
  dns_add_nameserver((FAR struct sockaddr *)&dnsaddr, sizeof(dnsaddr));

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name: wiznet_get_net
 *
 * Description:
 *   It will get network ip of mode.
 *
 * Output Parmeters:
 *   dev  : private data for wiznet driver access
 *   ip   : ip address info structure of interface
 *
 * Returned Value:
 *   0 on success, -1 in case of error.
 *
 ****************************************************************************/

int wiznet_get_net(FAR struct wiznet_dev_s *dev, wiznet_ipaddr_t *ip)
{
  wiz_netinfo netinfo;

  memset(&netinfo, 0, sizeof(wiz_netinfo));
  wizchip_getnetinfo(&netinfo);

  *((uint8_t *)&(ip->ip)+0) = netinfo.ip[0];
  *((uint8_t *)&(ip->ip)+1) = netinfo.ip[1];
  *((uint8_t *)&(ip->ip)+2) = netinfo.ip[2];
  *((uint8_t *)&(ip->ip)+3) = netinfo.ip[3];
  *((uint8_t *)&(ip->mask)+0) = netinfo.sn[0];
  *((uint8_t *)&(ip->mask)+1) = netinfo.sn[1];
  *((uint8_t *)&(ip->mask)+2) = netinfo.sn[2];
  *((uint8_t *)&(ip->mask)+3) = netinfo.sn[3];
  *((uint8_t *)&(ip->gw)+0) = netinfo.gw[0];
  *((uint8_t *)&(ip->gw)+1) = netinfo.gw[1];
  *((uint8_t *)&(ip->gw)+2) = netinfo.gw[2];
  *((uint8_t *)&(ip->gw)+3) = netinfo.gw[3];
  *((uint8_t *)&(ip->dns)+0) = dev->dns_server[0];
  *((uint8_t *)&(ip->dns)+1) = dev->dns_server[1];
  *((uint8_t *)&(ip->dns)+2) = dev->dns_server[2];
  *((uint8_t *)&(ip->dns)+3) = dev->dns_server[3];
  memmove(ip->mac, netinfo.mac, 6);

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name:  wiznet_bind
 *
 * Description:
 *   Equivalent of POSIX bind for WIZNET.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor returned by socket()
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   A 0 on success; -errno on error.
 *
 ****************************************************************************/

int wiznet_bind(int sockfd,
                FAR const struct sockaddr *addr, socklen_t addrlen)
{
  uint16_t port;

  port = ntohs(((struct sockaddr_in *)addr)->sin_port);
  set_sn_port(sockfd, port);

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name:  wiznet_accept
 *
 * Description:
 *   Equivalent of POSIX accept for WIZNET.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor returned by socket()
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *
 * Returned Value:
 *   A 0 on success; -errno on error.
 *
 ****************************************************************************/

int wiznet_accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
  uint8_t val;

  while ((val = get_sn_sr(sockfd)) != SOCK_ESTABLISHED)
    {
      if (val == SOCK_CLOSED)
        {
          errno = ENETDOWN;
          return -1;
        }

      wiznet_unlock_access(g_wizdev);
      nxsig_usleep(WIZNET_ACCEPT_WAIT_US);
      wiznet_lock_access(g_wizdev);
    }

  errno = 0;
  return 0;
}

/****************************************************************************
 * Name:  wiznet_check_interrupt
 *
 * Description:
 *   Check interrupted socket.
 *
 * Input Parameters:
 *   dev  : private data for wiznet driver access
 *
 * Returned Value:
 *   Positive value for success case, negative vale for error case.
 *
 ****************************************************************************/

int wiznet_check_interrupt(FAR struct wiznet_dev_s *dev)
{
  int cnt;

  for (cnt = 1; cnt < _WIZCHIP_SOCK_NUM_; cnt++)
    {
      if (get_sn_ir(cnt) & SN_IR_CHECK)
        {
          return cnt;
        }
    }

  return -1;
}

/****************************************************************************
 * Name:  wiznet_reset_interrupt
 *
 * Description:
 *   Check interrupted socket.
 *
 * Input Parameters:
 *   dev    : private data for wiznet driver access
 *   sockfd : socket descriptor returned by socket()
 *
 * Returned Value:
 *   Positive value for success case, negative vale for error case.
 *
 ****************************************************************************/

void wiznet_reset_interrupt(FAR struct wiznet_dev_s *dev, int sockfd)
{
  if ((sockfd > 0) && (sockfd < _WIZCHIP_SOCK_NUM_))
    {
      if (get_sn_rx_rsr(sockfd) == 0)
        {
          set_sn_ir(sockfd, SN_IR_RESET);
        }
    }
}

