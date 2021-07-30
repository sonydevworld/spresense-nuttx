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
#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

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
  /* Power on altair modem device */

  cxd56_gpio_config(ALT1250_SHUTDOWN, false);
  cxd56_gpio_write(ALT1250_SHUTDOWN, true);

  cxd56_gpio_config(ALT1250_LTE_POWER_BUTTON, false);
  cxd56_gpio_write(ALT1250_LTE_POWER_BUTTON, true);

  board_power_control(POWER_LTE, true);
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
  /* Power off Altair modem device */

  cxd56_gpio_write(ALT1250_SHUTDOWN, true);

  board_power_control(POWER_LTE, false);

  cxd56_gpio_write(ALT1250_SHUTDOWN, false);
  cxd56_gpio_write(ALT1250_LTE_POWER_BUTTON, false);
}

#endif

