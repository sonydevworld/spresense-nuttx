/****************************************************************************
 * arch/arm/include/cxd56xx/hostif.h
 *
 *   Copyright 2021 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H
#define __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Host interface maximum number of buffers */

#define MAX_BUFFER_NUM 32

/* Host interface buffer attributes */

#define HOSTIF_BUFF_ATTR_ADDR_OFFSET(n) (((n) & 0x3) << 4)
                                           /* 2 to the power of n */
#define HOSTIF_BUFF_ATTR_FIXLEN   (0 << 2) /* fixed length */
#define HOSTIF_BUFF_ATTR_VARLEN   (1 << 2) /* variable length */
#define HOSTIF_BUFF_ATTR_WRITE    (0 << 1) /* from target to host */
#define HOSTIF_BUFF_ATTR_READ     (1 << 1) /* from host to target */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Common buffer configuration */

struct hostif_buff_s
{
  uint16_t size;
  uint16_t flag;
};

/* I2C buffer configuration */

struct hostif_i2cconf_s
{
  int                  address; /* slave address */
  struct hostif_buff_s buff[MAX_BUFFER_NUM];
};

/* SPI buffer configuration */

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

/****************************************************************************
 * Name: hostif_i2cinitialize
 *
 * Description:
 *   Initialize the host interface for I2C slave
 *
 * Input Parameter:
 *   config - pointer to I2C buffer configuration
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_i2cinitialize(FAR struct hostif_i2cconf_s *config);

/****************************************************************************
 * Name: hostif_spiinitialize
 *
 * Description:
 *   Initialize the host interface for SPI slave
 *
 * Input Parameter:
 *   config - pointer to SPI buffer configuration
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_spiinitialize(FAR struct hostif_spiconf_s *config);

/****************************************************************************
 * Name: hostif_uninitialize
 *
 * Description:
 *   Uninitialize the host interface
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int hostif_uninitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_CXD56XX_HOSTIF_H */
