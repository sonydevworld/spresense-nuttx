/****************************************************************************
 * include/nuttx/video/imgdata.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_IMGDATA_H
#define __INCLUDE_NUTTX_VIDEO_IMGDATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Format definition for start_capture() and validate_frame_setting */

#define IMGDATA_FMT_MAX                  (2)
#define IMGDATA_FMT_MAIN                 (0)
#define IMGDATA_FMT_SUB                  (1)
#define IMGDATA_PIX_FMT_UYVY             (0)
#define IMGDATA_PIX_FMT_RGB565           (1)
#define IMGDATA_PIX_FMT_JPEG             (2)
#define IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG (3)
#define IMGDATA_PIX_FMT_SUBIMG_UYVY      (4)
#define IMGDATA_PIX_FMT_SUBIMG_RGB565    (5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* structure for validate_frame_setting() and start_capture() */

typedef struct imgdata_format_s
{
  uint16_t width;
  uint16_t height;
  uint32_t pixelformat;
} imgdata_format_t;

typedef struct imgdata_interval_s
{
  uint32_t numerator;
  uint32_t denominator;
} imgdata_interval_t;

typedef int (*imgdata_capture_t)(uint8_t result, uint32_t size);

/* Structure for Data Control I/F */

struct imgdata_ops_s
{
  CODE int (*init)(void);
  CODE int (*uninit)(void);

  CODE int (*validate_buf)(uint8_t *addr, uint32_t size);
  CODE int (*set_buf)(uint8_t *addr, uint32_t size);

  CODE int (*validate_frame_setting)(uint8_t nr_datafmts,
                                     FAR imgdata_format_t *datafmts,
                                     FAR imgdata_interval_t *interval);
  CODE int (*start_capture)(uint8_t nr_datafmts,
                            FAR imgdata_format_t *datafmts,
                            FAR imgdata_interval_t *interval,
                            FAR imgdata_capture_t callback);
  CODE int (*stop_capture)(void);
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Register image data operations. */

void imgdata_register(const FAR struct imgdata_ops_s *ops);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_IMGDATA_H */
