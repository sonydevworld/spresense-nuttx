/****************************************************************************
 * drivers/modem/alt1250/altmdm_spi.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_MODEM_ALT1250_ALTMDM_SPI_H
#define __DRIVERS_MODEM_ALT1250_ALTMDM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTSPI_PKT_STATUS_POS  (28)

#define ALTSPI_SLEEP_BIT    (1 << 28)
#define ALTSPI_RESET_BIT    (1 << 30)
#define ALTSPI_BUFFFULL_BIT (1 << 31)
#define ALTSPI_STATUSMASK   (0x0f << ALTSPI_PKT_STATUS_POS)

#define ALTSPI_PKT_TOTALSIZE_POS  (14)
#define ALTSPI_PKT_SIZEMASK ((1 << 14) - 1)

#define ALTSPI_PKT_WORDSIZE (4)
#define ALTSPI_PKT_HDRSIZE ALTSPI_PKT_WORDSIZE

#define ALTSPI_MAX_PKTSIZE  (2064)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct altmdm_spipkt_s
{
  uint32_t header;
  void *buffer;
  int buff_size;
} altmdm_spipkt_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline FAR void *get_pkt_buffer(FAR altmdm_spipkt_t *pkt)
{
  return pkt->buffer;
}

static inline int get_pkt_size(FAR altmdm_spipkt_t *pkt)
{
  return pkt->buff_size;
}

static inline int get_spipayload_maxsize(void)
{
  return ALTSPI_MAX_PKTSIZE;
}

static inline uint16_t pkt_total_size(FAR altmdm_spipkt_t *pkt)
{
  return (uint16_t)
    ((pkt->header >> ALTSPI_PKT_TOTALSIZE_POS) & ALTSPI_PKT_SIZEMASK);
}

static inline uint16_t pkt_actual_size(FAR altmdm_spipkt_t *pkt)
{
  return (uint16_t)(pkt->header & ALTSPI_PKT_SIZEMASK);
}

static inline int is_sleep_pkt(FAR altmdm_spipkt_t *pkt)
{
  return (pkt->header & ALTSPI_SLEEP_BIT);
}

static inline int is_reset_pkt(FAR altmdm_spipkt_t *pkt)
{
  return (pkt->header & ALTSPI_RESET_BIT);
}

static inline int has_sendrequest(FAR altmdm_spipkt_t *pkt)
{
  return (pkt->buffer != NULL);
}

static inline int is_buffer_full(FAR altmdm_spipkt_t *pkt)
{
  return (pkt->header & ALTSPI_BUFFFULL_BIT);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void altmdm_spipkt_init(FAR altmdm_spipkt_t *pkt);
void altmdm_set_spipkt_rxbuffer(FAR altmdm_spipkt_t *pkt, FAR void *buf,
  uint16_t sz);
void altmdm_set_spipkt_txbuffer(FAR altmdm_spipkt_t *pkt, FAR void *buf,
  uint16_t sz);
void altmdm_overwrite_body_size(FAR altmdm_spipkt_t *pkt, uint16_t sz);
void altmdm_set_sleeppkt(FAR altmdm_spipkt_t *pkt);
bool altmdm_is_valid_spipkt_header(FAR altmdm_spipkt_t *pkt);
bool altmdm_is_sleeppkt_ok(FAR altmdm_spipkt_t *pkt);
void altmdm_do_hdr_transaction(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower, FAR altmdm_spipkt_t *tx_pkt,
  FAR altmdm_spipkt_t *rx_pkt);
void altmdm_do_body_transaction(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower, FAR altmdm_spipkt_t *tx_pkt,
  FAR altmdm_spipkt_t *rx_pkt);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTMDM_SPI_H */
