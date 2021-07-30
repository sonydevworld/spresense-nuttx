/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H__
#define __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/modem/alt1250.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

compose_handler_t alt1250_composehdlr(uint32_t cmdid);
parse_handler_t alt1250_parsehdlr(uint16_t altcid, uint8_t altver);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_HDLR_H__ */
