/****************************************************************************
 * drivers/modem/alt1250/altmdm_event.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H__
#define __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct altmdm_event_s
{
  sem_t sem;
  uint32_t event;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int altmdm_event_init(FAR struct altmdm_event_s *evt);
int altmdm_event_destroy(FAR struct altmdm_event_s *evt);
uint32_t altmdm_event_wait(FAR struct altmdm_event_s *evt,
  uint32_t event, bool with_clear, int timeout_ms);
int altmdm_event_set(FAR struct altmdm_event_s *evt, uint32_t event);
int altmdm_event_clear(FAR struct altmdm_event_s *evt, uint32_t event);
uint32_t altmdm_event_refer(FAR struct altmdm_event_s *evt);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTMDM_EVENT_H__ */
