/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_wiznet.c
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

#if defined(CONFIG_NET_WIZNET) && \
  defined(CONFIG_CXD56_GPIO_IRQ) && defined(CONFIG_CXD56_SPI)

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/net/wiznet.h>

#include <arch/board/board.h>
#include <arch/board/cxd56_wiznet.h>

#include "cxd56_spi.h"
#include "cxd56_dmac.h"
#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_NET_WIZNET_MAX_PACKET_SIZE)
#  error CONFIG_NET_WIZNET_MAX_PACKET_SIZE is not set
#endif

#if defined(CONFIG_CXD56_WIZNET_SPI4)
#  define SPI_CH           (4)
#  if  defined(CONFIG_CXD56_WIZNET_SPI4_DMAC)
#    define DMA_TXCH       (2)
#    define DMA_RXCH       (3)
#    define DMA_TXCHCHG    (CXD56_DMA_PERIPHERAL_SPI4_TX)
#    define DMA_RXCHCFG    (CXD56_DMA_PERIPHERAL_SPI4_RX)
#  endif
#elif defined(CONFIG_CXD56_WIZNET_SPI5)
#  define SPI_CH           (5)
#  if  defined(CONFIG_CXD56_WIZNET_SPI5_DMAC)
#    define DMA_TXCH       (4)
#    define DMA_RXCH       (5)
#    define DMA_TXCHCHG    (CXD56_DMA_PERIPHERAL_SPI5_TX)
#    define DMA_RXCHCFG    (CXD56_DMA_PERIPHERAL_SPI5_RX)
#  endif
#else
#  error "Select CXD56 WIZnet SPI config to 4 or 5"
#endif

#define WIZNET_DELAY_RST   (1 * 1000)  /* ms */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void wiznet_irq_attach(bool attach, xcpt_t handler, FAR void *arg);
static void wiznet_irq_enable(bool enable);
static void wiznet_power_on(bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_devhandle = NULL;
static const struct wiznet_lower_s g_wiznet_lower =
{
  .attach  = wiznet_irq_attach,
  .enable  = wiznet_irq_enable,
  .poweron = wiznet_power_on,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wiznet_irq_attach
 ****************************************************************************/

static void wiznet_irq_attach(bool attach, xcpt_t handler, FAR void *arg)
{
  if (attach)
    {
      cxd56_gpioint_config(WIZNET_PIN_INT,
                           GPIOINT_LEVEL_LOW|GPIOINT_NOISE_FILTER_DISABLE,
                           handler, arg);
    }
  else
    {
      cxd56_gpioint_config(WIZNET_PIN_INT, 0, NULL, NULL);
    }
}

/****************************************************************************
 * Name: wiznet_irq_enable
 ****************************************************************************/

static void wiznet_irq_enable(bool enable)
{
  irqstate_t flags = spin_lock_irqsave();

  if (enable)
    {
      /* enable interrupt */

      cxd56_gpioint_enable(WIZNET_PIN_INT);
    }
  else
    {
      /* disable interrupt */

      cxd56_gpioint_disable(WIZNET_PIN_INT);
    }

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure the SPI pin
 *
 * Input Parameter:
 *   bus - SPI bus number to control
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void spi_pincontrol(int bus, bool on)
{
  switch (bus)
    {
#ifdef CONFIG_CXD56_SPI4
      case 4:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI4_GPIO);
          }
        break;
#endif /* CONFIG_CXD56_SPI4 */

#ifdef CONFIG_CXD56_SPI5
      case 5:
#ifdef CONFIG_CXD56_SPI5_PINMAP_EMMC
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_EMMCA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_EMMC */

#ifdef CONFIG_CXD56_SPI5_PINMAP_SDIO
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_SPI5);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SDIOA_GPIO);
          }
#endif /* CONFIG_CXD56_SPI5_PINMAP_SDIO */
        break;
#endif /* CONFIG_CXD56_SPI5 */
      default:
        break;
    }
}

/****************************************************************************
 * Name: wiznet_poweron
 *
 * Description:
 *   Power on the wiznet device on the board.
 *
 ****************************************************************************/

static void wiznet_power_on(bool on)
{
  if (on)
    {
      ninfo("Power on wiznet device..\n");

      /* enable the SPI pin */

      spi_pincontrol(SPI_CH, true);

      /* power on wiznet device */

      cxd56_gpio_write(WIZNET_PIN_RST, false);
      usleep(WIZNET_DELAY_RST);
      cxd56_gpio_write(WIZNET_PIN_RST, true);
    }
  else
    {
      ninfo("Power off wiznet device..\n");

      /* disable the SPI pin */

      spi_pincontrol(SPI_CH, false);

      /* power off wiznet device */

      cxd56_gpio_write(WIZNET_PIN_RST, false);
    }
}

#if defined(CONFIG_CXD56_WIZNET_USE_SWCS)
static void sw_spi_cs(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  /* Active low */

  cxd56_gpio_write(WIZNET_PIN_CS, selected);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_wiznet_initialize
 *
 * Description:
 *   Initialize wiznet device
 *
 ****************************************************************************/

int board_wiznet_initialize(FAR const char *devpath)
{
  FAR struct spi_dev_s *spi;
#if defined(CONFIG_CXD56_WIZNET_SPI4_DMAC) || \
    defined(CONFIG_CXD56_WIZNET_SPI5_DMAC)
  DMA_HANDLE            hdl;
  dma_config_t          conf;
#endif

#if defined(CONFIG_CXD56_WIZNET_USE_SWCS)
  FAR struct spi_ops_s *non_const_ops;
#endif

  ninfo("Initializing wiznet device..\n");

  if (!g_devhandle)
    {

      /* Initialize GPIO */

      cxd56_gpio_config(WIZNET_PIN_RST, false);

      /* Initialize spi deivce */

      spi = cxd56_spibus_initialize(SPI_CH);
      if (!spi)
        {
          nerr("ERROR: Failed to initialize spi%d.\n", SPI_CH);
          return -ENODEV;
        }

#if defined(CONFIG_CXD56_WIZNET_SPI4_DMAC) || \
    defined(CONFIG_CXD56_WIZNET_SPI5_DMAC)
      hdl = cxd56_dmachannel(DMA_TXCH,
                             CONFIG_NET_WIZNET_MAX_PACKET_SIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_TXCHCHG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_TX, hdl,
                              &conf);
        }

      hdl = cxd56_dmachannel(DMA_RXCH,
                             CONFIG_NET_WIZNET_MAX_PACKET_SIZE);
      if (hdl)
        {
          conf.channel_cfg = DMA_RXCHCFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(SPI_CH, CXD56_SPI_DMAC_CHTYPE_RX, hdl,
                              &conf);
        }
#endif

      spi_pincontrol(SPI_CH, true);

#if defined(CONFIG_CXD56_WIZNET_USE_SWCS)

      /* Set Chip Select PIN as GPIO */

      cxd56_gpio_config(WIZNET_PIN_CS, false);

      /* Override select method on SPI instance */

      non_const_ops = (struct spi_ops_s *)spi->ops;
      non_const_ops->select = sw_spi_cs;

#endif

      g_devhandle = wiznet_register(devpath, spi, &g_wiznet_lower);
      if (!g_devhandle)
        {
          nerr("ERROR: Failed to register wiznet driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}

#endif
