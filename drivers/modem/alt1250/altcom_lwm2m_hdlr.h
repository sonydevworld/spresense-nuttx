/****************************************************************************
 * drivers/modem/alt1250/altcom_lwm2m_hdlr.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H__
#define __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef int32_t (*lwm2mstub_hndl_t)(FAR uint8_t *, size_t,
                          FAR void **, size_t);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

lwm2mstub_hndl_t lwm2mstub_get_handler(FAR uint8_t **pktbuf, size_t *pktsz,
                                      uint32_t *lcmdid);

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_LWM2M_HDLR_H__ */
