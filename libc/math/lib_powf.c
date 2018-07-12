/****************************************************************************
 * libc/math/lib_powf.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatibile, MIT-style license:
 *
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float powf(float b, float e)
{
  float ret;
  float ip;

  if (e == 0.0F)
    {
      return 1.0F;
    }

  if ((b == -1.0F) && ((e == INFINITY) || (e == -INFINITY)))
    {
      return 1.0F;
    }

  if (b == 1.0F)
    {
      return b;
    }
  else if (b > 0.0F)
    {
      return expf(e * logf(b));
    }
  else
    {
      modff(e, &ip);

      if (b < 0.0F)
        {
          if (e == ip)
            {
              ret = expf(e * logf(-b));
              return ((int32_t)e % 2) ? -ret : ret;
            }
        }
      else if (b == 0.0F)
        {
          if (e > 0.0F)
            {
              return b;
            }
          else if (e < 0.0F)
            {
              if (e == ip)
                {
                  return (*(uint32_t *)&b) ? -INFINITY : INFINITY;
                }
            }
        }
    }
  return NAN;
}
