/****************************************************************************
 * arch/arm/include/cxd56xx/hostif.h
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

/**
 * @file hostif.h
 */

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H
#define __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H

/**
 * @defgroup hostif HostIF (Host Interface) driver
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/** @brief Host interface maximum number of buffers */

#define MAX_BUFFER_NUM 32

/** @brief Host interface buffer attributes */

#define HOSTIF_BUFF_ATTR_ADDR_OFFSET(n) (((n) & 0x3) << 4)
                                           /**< 2 to the power of n */
#define HOSTIF_BUFF_ATTR_FIXLEN   (0 << 2) /**< fixed length */
#define HOSTIF_BUFF_ATTR_VARLEN   (1 << 2) /**< variable length */
#define HOSTIF_BUFF_ATTR_WRITE    (0 << 1) /**< from target to host */
#define HOSTIF_BUFF_ATTR_READ     (1 << 1) /**< from host to target */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** @brief Common buffer configuration */

struct hostif_buff_s
{
  uint16_t size; /**< The size of buffer */
  uint16_t flag; /**< The attribute of buffer */
};

/** @brief I2C buffer configuration */

struct hostif_i2cconf_s
{
  int                  address; /**< slave address */
  struct hostif_buff_s buff[MAX_BUFFER_NUM];
};

/** @brief SPI buffer configuration */

struct hostif_spiconf_s
{
  struct hostif_buff_s buff[MAX_BUFFER_NUM];
};

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * Initialize the host interface for I2C slave
 *
 * @param [in] config - pointer to I2C buffer configuration
 *
 * @return Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_i2cinitialize(FAR struct hostif_i2cconf_s *config);

/**
 * Initialize the host interface for SPI slave
 *
 * @param [in] config - pointer to SPI buffer configuration
 *
 * @return Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_spiinitialize(FAR struct hostif_spiconf_s *config);

/**
 * Uninitialize the host interface
 *
 * @return Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_uninitialize(void);

/** @} hostif */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H */
