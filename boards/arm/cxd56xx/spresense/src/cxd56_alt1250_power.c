/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_alt1250_power.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_MODEM_ALT1250)

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/alt1250.h>
#include <arch/board/board.h>
#include <nuttx/signal.h>
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POWERON_INTERVAL_NSEC (100 * 1000 * 1000)
#define POWEROFF_INTERVAL_NSEC (100 * 1000)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_alt1250_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

void board_alt1250_poweron(void)
{
  struct timespec interval =
    {
      0, POWERON_INTERVAL_NSEC
    };

  /* Power on altair modem device */

  board_power_control(POWER_LTE, true);

  nxsig_nanosleep(&interval, NULL);

  cxd56_gpio_write(ALT1250_SHUTDOWN, true);
  cxd56_gpio_write(ALT1250_LTE_POWER_BUTTON, true);
}

/****************************************************************************
 * Name: board_alt1250_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_alt1250_poweroff(void)
{
  struct timespec interval =
    {
      0, POWEROFF_INTERVAL_NSEC
    };

  /* Power off Altair modem device */

  cxd56_gpio_write(ALT1250_LTE_POWER_BUTTON, false);
  cxd56_gpio_write(ALT1250_SHUTDOWN, false);

  nxsig_nanosleep(&interval, NULL);

  board_power_control(POWER_LTE, false);

  /* Output disable */

  cxd56_gpio_config(ALT1250_LTE_POWER_BUTTON, false);
  cxd56_gpio_config(ALT1250_SHUTDOWN, false);
}

/****************************************************************************
 * Name: board_alt1250_reset
 *
 * Description:
 *   Reset the Altair modem device on the board.
 *
 ****************************************************************************/

void board_alt1250_reset(void)
{
  struct timespec interval =
    {
      0, POWEROFF_INTERVAL_NSEC
    };

  /* Reset Altair modem device */

  cxd56_gpio_write(ALT1250_SHUTDOWN, false);

  nxsig_nanosleep(&interval, NULL);

  cxd56_gpio_write(ALT1250_SHUTDOWN, true);
}

#endif

