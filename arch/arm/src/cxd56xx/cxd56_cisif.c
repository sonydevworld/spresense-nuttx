/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cisif.c
 *
 *   Copyright 2018, 2020 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/chip/cisif.h>
#include <nuttx/video/video.h>
#include <nuttx/video/video_halif.h>
#include "up_arch.h"

#include "cxd56_clock.h"
#include "hardware/cxd56_cisif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To see the interrupt timing of Vsync */

/* #define CISIF_INTR_TRACE */

/* #define CISIF_DBG_CONTI_CAP */

#define YUV_VSIZE_STEP (1)
#define YUV_HSIZE_STEP (1)
#define YUV_VSIZE_MIN  (64)
#define YUV_HSIZE_MIN  (96)
#define YUV_VSIZE_MAX  (360)
#define YUV_HSIZE_MAX  (480)

#define JPG_VSIZE_STEP (1)
#define JPG_HSIZE_STEP (1)
#define JPG_VSIZE_MIN  (64)
#define JPG_HSIZE_MIN  (96)
#define JPG_VSIZE_MAX  (1944)
#define JPG_HSIZE_MAX  (2592)

#define JPG_INT_ALL   (JPG_ERR_STATUS_INT | \
                       JPG_MEM_OVF_INT    | \
                       JPG_FIFO_OVF_INT   | \
                       JPG_AXI_TRERR_INT  | \
                       JPG_MARKER_ERR_INT | \
                       JPG_AXI_TRDN_INT)

#define YCC_INT_ALL   (YCC_MEM_OVF_INT    | \
                       YCC_FIFO_OVF_INT   | \
                       YCC_AXI_TRERR_INT  | \
                       YCC_MARKER_ERR_INT | \
                       SIZE_UNDER_INT     | \
                       SIZE_OVER_INT      | \
                       YCC_AXI_TRDN_INT)

/* YUV data size with frame v * h */

#define YUV_SIZE(v, h) (v * h * 2)

/* Check Buffer address alignment */

#define CISIF_BUFADDR_ALIGNMENT            (32)
#define ILLEGAL_BUFADDR_ALIGNMENT(addr)    (((addr) == NULL) ||  \
                                            (((uint32_t)(addr) % \
                                              CISIF_BUFADDR_ALIGNMENT) != 0))

#ifdef CONFIG_CXD56_CISIF_DEBUG
#define ciferr    _err
#define cifwarn   _warn
#define cifinfo   _info
#else
#define ciferr(x...)
#define cifwarn(x...)
#define cifinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum state_e
{
  STATE_STANDBY,
  STATE_READY,
  STATE_CAPTURE,
};

typedef enum state_e state_t;

typedef void (*intc_func_table)(uint8_t code);

typedef void (*notify_callback_t)(uint8_t code, uint32_t size, uint32_t addr);
typedef void (*comp_callback_t)(uint8_t code, uint32_t size, uint32_t addr);

struct cisif_init_yuv_param_s
{
  uint16_t          hsize;
  uint16_t          vsize;
  uint32_t          notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_init_yuv_param_s cisif_init_yuv_param_t;

struct cisif_init_jpeg_param_s
{
  uint32_t notify_size;
  notify_callback_t notify_func;
};

typedef struct cisif_init_jpeg_param_s cisif_init_jpeg_param_t;

struct cisif_sarea_s
{
  uint8_t *strg_addr;
  uint32_t strg_size;
};

typedef struct cisif_sarea_s cisif_sarea_t;

struct cisif_param_s
{
  uint32_t                format;
  cisif_init_yuv_param_t  yuv_param;
  cisif_init_jpeg_param_t jpg_param;
  cisif_sarea_t           sarea;
};

typedef struct cisif_param_s cisif_param_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR static void *g_cisif_video_private = NULL;
static state_t g_state = STATE_STANDBY;
static uint32_t g_storage_addr = 0;

static notify_callback_t g_jpg_notify_callback_func;
static notify_callback_t g_ycc_notify_callback_func;

static bool     g_jpgint_receive;
static bool     g_errint_receive;

#ifdef CISIF_INTR_TRACE
static uint32_t g_cisif_vint_count = 0;
static uint32_t g_cisif_vint_count_max = 0;
static uint32_t g_cisif_time_start;
static uint32_t g_cisif_time_stop;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cisif_vs_int(uint8_t code);
static void cisif_ycc_axi_trdn_int(uint8_t code);
static void cisif_ycc_nstorage_int(uint8_t code);
static void cisif_jpg_axi_trdn_int(uint8_t code);
static void cisif_jpg_nstorage_int(uint8_t code);
static void cisif_ycc_err_int(uint8_t code);
static void cisif_jpg_err_int(uint8_t code);

static void     cisif_reg_write(uint16_t reg, uint32_t val);
static uint32_t cisif_reg_read(uint16_t reg);

static int cisif_check_param(cisif_param_t *p);
static int cisif_set_yuv_param(cisif_param_t *p);
static int cisif_set_jpg_param(cisif_param_t *p);

static int cisif_check_sarea(void *s);
static int cisif_set_yuv_sarea(void *s);
static int cisif_set_jpg_sarea(void *s);
static int cisif_set_intlev_sarea(void *s, uint32_t yuv_size);
static int cisif_intc_handler(int irq, FAR void *context, FAR void *arg);

/* video image data operations */

static int cxd56_cisif_open(FAR void *video_private);
static int cxd56_cisif_close(void);
static int cxd56_cisif_start_dma(FAR struct v4l2_format *format,
                                 uint32_t bufaddr,
                                 uint32_t bufsize);
static int cxd56_cisif_cancel_dma(void);
static int cxd56_cisif_set_dmabuf(uint32_t bufaddr, uint32_t bufsize);
static int cxd56_cisif_get_range_of_framesize(FAR struct v4l2_frmsizeenum
                                              *frmsize);
static int cxd56_cisif_chk_pixelformat(uint32_t pixelformat,
                                       uint32_t subimg_pixelformat);
static int cxd56_cisif_try_format(FAR struct v4l2_format *format);

const intc_func_table g_intcomp_func[] =
  {
    cisif_vs_int,            /* VS_INT */
    NULL,                    /* EOY_INT */
    NULL,                    /* SOY_INT */
    NULL,                    /* EOI_INT */
    NULL,                    /* SOI_INT */
    NULL,                    /* YCC_VACT_END_INT */
    NULL,                    /* JPG_VACT_END_INT */
    cisif_ycc_axi_trdn_int,  /* YCC_AXI_TRDN_INT */
    cisif_ycc_nstorage_int,  /* YCC_NSTORAGE_INT */
    NULL,                    /* YCC_DAREA_END_INT */
    cisif_jpg_axi_trdn_int,  /* JPG_AXI_TRDN_INT */
    cisif_jpg_nstorage_int,  /* JPG_NSTORAGE_INT */
    NULL,                    /* JPG_DAREA_END_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    NULL,                    /* VLATCH_INT */
    cisif_ycc_err_int,       /* SIZE_OVER_INT */
    cisif_ycc_err_int,       /* SIZE_UNDER_INT */
    cisif_ycc_err_int,       /* YCC_MARKER_ERR_INT */
    cisif_ycc_err_int,       /* YCC_AXI_TRERR_INT */
    cisif_ycc_err_int,       /* YCC_FIFO_OVF_INT */
    cisif_ycc_err_int,       /* YCC_MEM_OVF_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    cisif_jpg_err_int,       /* JPG_MARKER_ERR_INT */
    cisif_jpg_err_int,       /* JPG_AXI_TRERR_INT */
    cisif_jpg_err_int,       /* JPG_FIFO_OVF_INT */
    cisif_jpg_err_int,       /* JPG_MEM_OVF_INT */
    cisif_jpg_err_int,       /* JPG_ERR_STATUS_INT */
  };

const struct video_imgdata_ops_s g_cxd56_cisif_ops =
  {
    .open                   = cxd56_cisif_open,
    .close                  = cxd56_cisif_close,
    .start_dma              = cxd56_cisif_start_dma,
    .set_dmabuf             = cxd56_cisif_set_dmabuf,
    .cancel_dma             = cxd56_cisif_cancel_dma,
    .get_range_of_framesize = cxd56_cisif_get_range_of_framesize,
    .try_format             = cxd56_cisif_try_format,
    .chk_pixelformat        = cxd56_cisif_chk_pixelformat,
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CISIF_INTR_TRACE
static uint64_t cisif_get_msec_time(void)
{
  struct timespec tp;

  if (clock_gettime(CLOCK_REALTIME, &tp))
    {
      return 0;
    }

  return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}

static void cisif_trace_time_start(void)
{
  g_cisif_time_start = (uint32_t)cisif_get_msec_time();
}

static void cisif_trace_time_stop(char *str)
{
  g_cisif_time_stop = (uint32_t)cisif_get_msec_time();
  printf("%s:%d[ms]\n", str, (uint32_t)(g_cisif_time_stop -
                               g_cisif_time_start));
}

void cisif_intrtrace_start(int max)
{
  g_cisif_vint_count_max = max;
  g_cisif_vint_count = 0;
  cisif_trace_time_start();
}
#endif

/****************************************************************************
 * cisif_vs_int
 ****************************************************************************/

static void cisif_vs_int(uint8_t code)
{
#ifdef CISIF_INTR_TRACE
  if (g_cisif_vint_count < g_cisif_vint_count_max)
    {
      cisif_trace_time_stop("cisif_vs_int");
      cisif_trace_time_start();
      g_cisif_vint_count++;
    }
  else
    {
      g_cisif_vint_count_max = 0;
    }
#endif

  switch (g_state)
    {
      case STATE_STANDBY:
        cifinfo("invalid state\n");
        break;

      case STATE_READY:
        g_errint_receive = false;
        break;

      case STATE_CAPTURE:
        g_errint_receive = false;
        break;

      default:
        cifinfo("invalid state\n");
        break;
    }
}

/****************************************************************************
 * cisif_callback_for_intlev
 ****************************************************************************/

static void cisif_callback_for_intlev(uint8_t code)
{
  uint32_t      size;
  uint32_t      yuv_size;

  if (!g_jpgint_receive)
    {
      /* In either YUV or JPEG is not received,
       * wait receiving.
       */

      g_jpgint_receive = true;
      return;
    }

  /* Read received data size */

  yuv_size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  size     = yuv_size + cisif_reg_read(CISIF_JPG_DSTRG_CONT);

  /* Notify and get next addr */

  video_common_notify_dma_done(0, size, g_cisif_video_private);

  g_jpgint_receive = false;

  cisif_reg_write(CISIF_EXE_CMD, 1);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);

  return;
}

/****************************************************************************
 * cisif_ycc_axi_trdn_int
 ****************************************************************************/

static void cisif_ycc_axi_trdn_int(uint8_t code)
{
  uint32_t size;
  uint32_t cisif_mode;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_axi_trdn_int");
#endif

  if (g_errint_receive)
    {
      /* In error occurred case in the same frame, ignore */

      cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
      return;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);
  if (cisif_mode == MODE_INTLEV_TRS_EN)
    {
      /* In JPEG + YUV format case */

      cisif_callback_for_intlev(code);
    }
  else
    {
      size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
      video_common_notify_dma_done(0, size, g_cisif_video_private);
      cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
    }
}

/****************************************************************************
 * cisif_ycc_nstorage_int
 ****************************************************************************/

static void cisif_ycc_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_ycc_notify_callback_func(0, size, g_storage_addr);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, size);
}

/****************************************************************************
 * cisif_jpg_axi_trdn_int
 ****************************************************************************/

static void cisif_jpg_axi_trdn_int(uint8_t code)
{
  uint32_t size;
  uint32_t cisif_mode;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_axi_trdn_int");
#endif

  if (g_errint_receive)
    {
      /* In error occurred case in the same frame, ignore */

      cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
      return;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);

  if (cisif_mode == MODE_INTLEV_TRS_EN)
    {
      /* In JPEG + YUV format case */

      cisif_callback_for_intlev(code);
    }
  else
    {
      size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
      video_common_notify_dma_done(0, size, g_cisif_video_private);
      cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
    }
}

/****************************************************************************
 * cisif_jpg_nstorage_int
 ****************************************************************************/

static void cisif_jpg_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);

  g_jpg_notify_callback_func(0, size, g_storage_addr);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, size);
}

/****************************************************************************
 * cisif_ycc_err_int
 ****************************************************************************/

static void cisif_ycc_err_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_ycc_err_int");
#endif

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  video_common_notify_dma_done(code, size, g_cisif_video_private);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_jpg_err_int
 ****************************************************************************/

static void cisif_jpg_err_int(uint8_t code)
{
  uint32_t size;

#ifdef CISIF_INTR_TRACE
  cisif_trace_time_stop("cisif_jpg_err_int");
#endif

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  video_common_notify_dma_done(code, size, g_cisif_video_private);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
  g_errint_receive = true;
}

/****************************************************************************
 * cisif_intc_handler
 ****************************************************************************/

static int cisif_intc_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t value;
  uint32_t enable;
  uint8_t  index;

  value = cisif_reg_read(CISIF_INTR_STAT);
  cisif_reg_write(CISIF_INTR_CLEAR, value & ALL_CLEAR_INT);
  cifinfo("int stat %08x\n", value);

  enable = cisif_reg_read(CISIF_INTR_ENABLE);
  value = (value & enable);

  for (index = 0;
       index < sizeof(g_intcomp_func) / sizeof(g_intcomp_func[0]);
       index++)
    {
      if ((value & (1 << index)) != 0)
        {
          g_intcomp_func[index](index);
        }
    }

  return OK;
}

/****************************************************************************
 * cisif_reg_write
 ****************************************************************************/

static void cisif_reg_write(uint16_t reg, uint32_t val)
{
  putreg32(val, CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * cisif_reg_read
 ****************************************************************************/

static uint32_t cisif_reg_read(uint16_t reg)
{
  return getreg32(CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * cisif_check_param
 ****************************************************************************/

static int cisif_check_param(cisif_param_t *p)
{
  if (p == NULL)
    {
      return -EINVAL;
    }

  switch (p->format)
    {
      case V4L2_PIX_FMT_UYVY:
      case V4L2_PIX_FMT_JPEG:
      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        break;

      default:
        return -EINVAL;
    }

  if (p->format != V4L2_PIX_FMT_JPEG)
    {
      if (p->yuv_param.hsize < YUV_HSIZE_MIN ||
          p->yuv_param.hsize > YUV_HSIZE_MAX ||
          p->yuv_param.vsize < YUV_VSIZE_MIN ||
          p->yuv_param.vsize > YUV_VSIZE_MAX)
        {
          return -EINVAL;
        }

      if (p->yuv_param.notify_func != NULL)
        {
          if (p->yuv_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  if (p->format != V4L2_PIX_FMT_UYVY)
    {
      if (p->jpg_param.notify_func != NULL)
        {
          if (p->jpg_param.notify_size == 0)
            {
              return -EINVAL;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * cisif_check_sarea
 ****************************************************************************/

static int cisif_check_sarea(void *s)
{
  if (s == NULL)
    {
      return -EINVAL;
    }

  cisif_sarea_t *ss = (cisif_sarea_t *)s;
  if (ILLEGAL_BUFADDR_ALIGNMENT(ss->strg_addr) ||
      ss->strg_size == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * cisif_set_yuvparam
 ****************************************************************************/

static int cisif_set_yuv_param(cisif_param_t *p)
{
  uint32_t act_size = 0;

  act_size = (p->yuv_param.vsize & 0x1ff) << 16;
  act_size |= p->yuv_param.hsize & 0x1ff;

  cisif_reg_write(CISIF_ACT_SIZE, act_size);
  cisif_reg_write(CISIF_CIS_SIZE, act_size);

  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_NSTRG_SIZE, (p->yuv_param.notify_size
                                                 & 0xffffffe0));

  g_ycc_notify_callback_func = p->yuv_param.notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_yuvsarea
 ****************************************************************************/

static int cisif_set_yuv_sarea(void *s)
{
  cisif_sarea_t *ss = (cisif_sarea_t *)s;

  /* must align 32 bytes */

  cisif_reg_write(CISIF_YCC_DAREA_SIZE, (ss->strg_size & 0xffffffe0));
  cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)ss->strg_addr);

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_param
 ****************************************************************************/

static int cisif_set_jpg_param(cisif_param_t *p)
{
  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_NSTRG_SIZE, (p->jpg_param.notify_size
                                               & 0xffffffe0));

  g_jpg_notify_callback_func = p->jpg_param.notify_func;

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_jpg_sarea(void *s)
{
  cisif_sarea_t *ss = (cisif_sarea_t *)s;

  /* must align 32 bytes */

  cisif_reg_write(CISIF_JPG_DAREA_SIZE, (ss->strg_size & 0xffffffe0));
  cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)ss->strg_addr);

  return OK;
}

/****************************************************************************
 * cisif_set_jpg_sarea
 ****************************************************************************/

static int cisif_set_intlev_sarea(void *s, uint32_t yuv_size)
{
  cisif_sarea_t *sarea = (cisif_sarea_t *)s;
  cisif_sarea_t sarea_int;

  if (sarea->strg_size < yuv_size)
    {
      return -EINVAL;
    }

  /* Set for YUV */

  sarea_int.strg_addr = sarea->strg_addr;
  sarea_int.strg_size = yuv_size;
  cisif_set_yuv_sarea(&sarea_int);

  /* Set for JPEG */

  sarea_int.strg_addr = sarea->strg_addr + yuv_size;
  sarea_int.strg_size = sarea->strg_size - yuv_size;

  cisif_set_jpg_sarea(&sarea_int);

  return OK;
}

/****************************************************************************
 * cisif_chk_jpgfrmsize
 ****************************************************************************/

static int cisif_chk_jpgfrmsize(int w, int h)
{
  if ((w < JPG_HSIZE_MIN) ||
      (w > JPG_HSIZE_MAX))
    {
      return -EINVAL;
    }

  if ((h < JPG_VSIZE_MIN) ||
      (h > JPG_VSIZE_MAX))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * cisif_chk_yuvfrmsize
 ****************************************************************************/

static int cisif_chk_yuvfrmsize(int w, int h)
{
  if ((w < YUV_HSIZE_MIN) ||
      (w > YUV_HSIZE_MAX))
    {
      return -EINVAL;
    }

  if ((h < YUV_VSIZE_MIN) ||
      (h > YUV_VSIZE_MAX))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * cxd56_cisif_open
 ****************************************************************************/

static int cxd56_cisif_open(FAR void *video_private)
{
  if (g_state != STATE_STANDBY)
    {
      return -EPERM;
    }

  /* enable CISIF clock */

  cxd56_img_cisif_clock_enable();

  /* disable CISIF interrupt */

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* attach interrupt handler */

  irq_attach(CXD56_IRQ_CISIF, cisif_intc_handler, NULL);

  /* enable CISIF irq  */

  up_enable_irq(CXD56_IRQ_CISIF);

#ifdef CISIF_INTR_TRACE
  cisif_reg_write(CISIF_INTR_ENABLE, VS_INT);
#endif

  g_state = STATE_READY;
  g_cisif_video_private = video_private;
  return OK;
}

/****************************************************************************
 * cxd56_cisif_close
 ****************************************************************************/

static int cxd56_cisif_close(void)
{
  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  /* disable CISIF irq  */

  up_disable_irq(CXD56_IRQ_CISIF);

  /* detach interrupt handler */

  irq_detach(CXD56_IRQ_CISIF);

  /* disable CISIF interrupt */

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);

  /* disable CISIF clock */

  cxd56_img_cisif_clock_disable();

  g_state = STATE_STANDBY;
  g_cisif_video_private = NULL;

  return OK;
}

/****************************************************************************
 * cxd56_cisif_start_dma
 ****************************************************************************/

static int cxd56_cisif_start_dma(FAR struct v4l2_format *format,
                                 uint32_t bufaddr,
                                 uint32_t bufsize)
{
  cisif_param_t param = {0};
  cisif_sarea_t sarea = {0};
  uint32_t cisif_mode;
  uint32_t interrupts = VS_INT;
  int ret;

  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  param.format = format->fmt.pix.pixelformat;
  if (param.format != V4L2_PIX_FMT_JPEG)
    {
      if (param.format == V4L2_PIX_FMT_UYVY)
        {
          param.yuv_param.hsize = format->fmt.pix.width;
          param.yuv_param.vsize = format->fmt.pix.height;
        }
      else
        {
          param.yuv_param.hsize = format->fmt.pix.subimg_width;
          param.yuv_param.vsize = format->fmt.pix.subimg_height;
        }
    }

  ret = cisif_check_param(&param);
  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);

  sarea.strg_addr = (uint8_t *)bufaddr;
  sarea.strg_size = bufsize;
  ret = cisif_check_sarea(&sarea);
  if (ret != OK)
    {
      return ret;
    }

  switch (param.format)
    {
      case V4L2_PIX_FMT_UYVY:
        cisif_set_yuv_param(&param);
        cisif_set_yuv_sarea(&sarea);

        cisif_mode = MODE_YUV_TRS_EN;
        interrupts |= YCC_INT_ALL;
        break;

      case V4L2_PIX_FMT_JPEG:
        cisif_set_jpg_param(&param);
        cisif_set_jpg_sarea(&sarea);

        cisif_mode = MODE_JPG_TRS_EN;
        interrupts |= JPG_INT_ALL;
        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:
        cisif_set_yuv_param(&param);
        cisif_set_jpg_param(&param);

        cisif_set_intlev_sarea(&sarea,
                               YUV_SIZE(param.yuv_param.vsize,
                               param.yuv_param.hsize));

        cisif_mode = MODE_INTLEV_TRS_EN;
        interrupts |= YCC_INT_ALL | JPG_INT_ALL;
        g_jpgint_receive = false;
        break;

      default:
        return -EINVAL;
    }

  g_storage_addr       = (uint32_t)sarea.strg_addr;

  g_state = STATE_CAPTURE;

  if (g_ycc_notify_callback_func != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_jpg_notify_callback_func != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_MODE, cisif_mode);
  cisif_reg_write(CISIF_INTR_CLEAR, interrupts);
  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

  cisif_reg_write(CISIF_DIN_ENABLE, 1);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

static int cxd56_cisif_cancel_dma(void)
{
  g_state = STATE_READY;
  cisif_reg_write(CISIF_DIN_ENABLE, 0);
  cisif_reg_write(CISIF_INTR_DISABLE, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

static int cxd56_cisif_set_dmabuf(uint32_t bufaddr, uint32_t bufsize)
{
  int      ret;
  uint32_t cisif_mode;
  uint32_t yuv_regsize;
  uint32_t yuv_hsize;
  uint32_t yuv_vsize;
  cisif_sarea_t sarea = {0};

  sarea.strg_addr = (uint8_t *)bufaddr;
  sarea.strg_size = bufsize;
  ret = cisif_check_sarea(&sarea);
  if (ret != OK)
    {
      return ret;
    }

  cisif_mode = cisif_reg_read(CISIF_MODE);

  switch (cisif_mode)
    {
      case MODE_YUV_TRS_EN:
        ret = cisif_set_yuv_sarea(&sarea);
        break;

      case MODE_JPG_TRS_EN:
        ret = cisif_set_jpg_sarea(&sarea);
        break;

      default: /* MODE_INTLEV_TRS_EN */

        /* Get YUV frame size information */

        yuv_regsize =  cisif_reg_read(CISIF_ACT_SIZE);
        yuv_vsize = (yuv_regsize >> 16) & 0x1ff;
        yuv_hsize = yuv_regsize & 0x01ff;

        ret = cisif_set_intlev_sarea(&sarea,
                                     YUV_SIZE(yuv_vsize, yuv_hsize));
        break;
    }

  if (ret != OK)
    {
      return ret;
    }

  cisif_reg_write(CISIF_EXE_CMD, 1);
  g_storage_addr = (uint32_t)sarea.strg_addr;

  return ret;
}

static int cxd56_cisif_get_range_of_framesize(FAR struct v4l2_frmsizeenum
                                             *frmsize)
{
  int ret = OK;

  if (frmsize == NULL)
    {
      return -EINVAL;
    }

  if (frmsize->index != 0)
    {
      return -EINVAL;
    }

  switch (frmsize->pixel_format)
    {
      case V4L2_PIX_FMT_UYVY:                /* YUV 4:2:2 */
        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = YUV_HSIZE_MIN;
        frmsize->stepwise.max_width          = YUV_HSIZE_MAX;
        frmsize->stepwise.step_width         = YUV_HSIZE_STEP;
        frmsize->stepwise.min_height         = YUV_VSIZE_MIN;
        frmsize->stepwise.max_height         = YUV_VSIZE_MAX;
        frmsize->stepwise.step_height        = YUV_VSIZE_STEP;

        break;

      case V4L2_PIX_FMT_JPEG:                /* JPEG */
        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = JPG_HSIZE_MIN;
        frmsize->stepwise.max_width          = JPG_HSIZE_MAX;
        frmsize->stepwise.step_width         = JPG_HSIZE_STEP;
        frmsize->stepwise.min_height         = JPG_VSIZE_MIN;
        frmsize->stepwise.max_height         = JPG_VSIZE_MAX;
        frmsize->stepwise.step_height        = JPG_VSIZE_STEP;

        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:    /* JPEG + YUV 4:2:2 */
        if (frmsize->subimg_pixel_format != V4L2_PIX_FMT_UYVY)
          {
            /* Unsupported pixel format */

            return -EINVAL;
          }

        frmsize->type                        = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->stepwise.min_width          = JPG_HSIZE_MIN;
        frmsize->stepwise.max_width          = JPG_HSIZE_MAX;
        frmsize->stepwise.step_width         = JPG_HSIZE_STEP;
        frmsize->stepwise.min_height         = JPG_VSIZE_MIN;
        frmsize->stepwise.max_height         = JPG_VSIZE_MAX;
        frmsize->stepwise.step_height        = JPG_VSIZE_STEP;

        frmsize->subimg_type                 = V4L2_FRMSIZE_TYPE_STEPWISE;
        frmsize->subimg.stepwise.min_width   = YUV_HSIZE_MIN;
        frmsize->subimg.stepwise.max_width   = YUV_HSIZE_MAX;
        frmsize->subimg.stepwise.step_width  = YUV_HSIZE_STEP;
        frmsize->subimg.stepwise.min_height  = YUV_VSIZE_MIN;
        frmsize->subimg.stepwise.max_height  = YUV_VSIZE_MAX;
        frmsize->subimg.stepwise.step_height = YUV_VSIZE_STEP;

        break;

      default: /* Unsupported pixel format */

        return -EINVAL;
    }

  return ret;
}

static int cxd56_cisif_chk_pixelformat(uint32_t pixelformat,
                                       uint32_t subimg_pixelformat)
{
  switch (pixelformat)
    {
      case V4L2_PIX_FMT_UYVY:

        return OK;

      case V4L2_PIX_FMT_JPEG:

        return OK;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:

        if (subimg_pixelformat == V4L2_PIX_FMT_UYVY)
          {
            return OK;
          }
        else
          {
            return -EINVAL;
          }

      default :

        return -EINVAL;
    }

  return OK;
}

static int cxd56_cisif_try_format(FAR struct v4l2_format *format)
{
  int ret = OK;

  switch (format->fmt.pix.pixelformat)
    {
      case V4L2_PIX_FMT_UYVY:                /* YUV 4:2:2 */

        ret = cisif_chk_yuvfrmsize(format->fmt.pix.width,
                                   format->fmt.pix.height);
        break;

      case V4L2_PIX_FMT_JPEG:                /* JPEG */

        ret = cisif_chk_jpgfrmsize(format->fmt.pix.width,
                                   format->fmt.pix.height);
        break;

      case V4L2_PIX_FMT_JPEG_WITH_SUBIMG:    /* JPEG + YUV 4:2:2 */

        if (format->fmt.pix.subimg_pixelformat != V4L2_PIX_FMT_UYVY)
          {
            /* Unsupported pixel format */

            return -EINVAL;
          }

        ret = cisif_chk_jpgfrmsize(format->fmt.pix.width,
                                   format->fmt.pix.height);
        if (ret != OK)
          {
            return ret;
          }

        ret = cisif_chk_yuvfrmsize(format->fmt.pix.subimg_width,
                                   format->fmt.pix.subimg_height);
        break;

      default: /* Unsupported pixel format */

        return -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * cxd56_cisif_initialize
 ****************************************************************************/

const FAR struct video_imgdata_ops_s *cxd56_cisif_initialize(void)
{
  return &g_cxd56_cisif_ops;
}

