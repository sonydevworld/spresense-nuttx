/****************************************************************************
 * drivers/modem/alt1250/altmdm.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTMDM_H__
#define __DRIVERS_MODEM_ALT1250_ALTMDM_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/modem/alt1250.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALTMDM_RETURN_RESET_V1  (-1)
#define ALTMDM_RETURN_NOTREADY  (-2)
#define ALTMDM_RETURN_CANCELED  (-3)
#define ALTMDM_RETURN_RESET_V4  (-4)
#define ALTMDM_RETURN_RESET_PKT (-5)
#define ALTMDM_RETURN_EXIT      (-6)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int altmdm_init(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower);
int altmdm_fin(void);
int altmdm_read(FAR uint8_t *buff, int sz);
int altmdm_write(FAR uint8_t *buff, int sz);
int altmdm_take_wlock(void);
int altmdm_give_wlock(void);
int altmdm_poweron(void);
int altmdm_poweroff(void);
int altmdm_reset(void);
uint32_t altmdm_get_reset_reason(void);
uint8_t altmdm_get_protoversion(void);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTMDM_H__ */
