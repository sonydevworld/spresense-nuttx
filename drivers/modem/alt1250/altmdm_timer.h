/****************************************************************************
 * drivers/modem/alt1250/altmdm_timer.h
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

#ifndef __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H__
#define __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <time.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

timer_t altmdm_timer_start(int first_ms, int interval_ms,
  FAR _sa_sigaction_t handler, FAR void *ptr_param);
int altmdm_timer_restart(timer_t timerid, int first_ms, int interval_ms);
int altmdm_timer_is_running(timer_t timerid);
void altmdm_timer_stop(timer_t timerid);

#endif  /* __DEVICES_MODEM_ALT1250_ALTMDM_TIMER_H__ */
