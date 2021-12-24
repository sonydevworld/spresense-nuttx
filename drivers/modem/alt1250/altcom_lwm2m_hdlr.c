/****************************************************************************
 * drivers/modem/alt1250/altcom_lwm2m_hdlr.c
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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/wireless/lte/lte_ioctl.h>

#include "altcom_lwm2m_hdlr.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int32_t read_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t write_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t exec_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t start_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t stop_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t fwupdate_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);
static int32_t server_op_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct urc_hdltbl_s
{
  const char *head;
  uint32_t lcmdid;
  lwm2mstub_hndl_t hdlr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct urc_hdltbl_s urc_idhandles[] =
{
  { "%LWM2MOBJCMDU: \"READ\"",
    LTE_CMDID_LWM2M_READ_EVT, read_request_hndl },

  { "%LWM2MOBJCMDU: \"WRITE\"",
    LTE_CMDID_LWM2M_WRITE_EVT, write_request_hndl },

  { "%LWM2MOBJCMDU: \"EXE\"",
    LTE_CMDID_LWM2M_EXEC_EVT, exec_request_hndl },

  { "%LWM2MOBJCMDU: \"OBSERVE_START\"",
    LTE_CMDID_LWM2M_OVSTART_EVT, start_ov_request_hndl },

  { "%LWM2MOBJCMDU: \"OBSERVE_STOP\"",
    LTE_CMDID_LWM2M_OVSTOP_EVT, stop_ov_request_hndl },

  { "%LWM2MOPEV: ",
    LTE_CMDID_LWM2M_SERVEROP_EVT, server_op_notice_hndl },

  { "%LWM2MEV: ",
    LTE_CMDID_LWM2M_FWUP_EVT, fwupdate_notice_hndl },

  { NULL,  0, NULL },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int32_t read_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return -1;
}

static int32_t write_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return -1;
}

static int32_t exec_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return -1;
}

static int32_t start_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return -1;
}

static int32_t stop_ov_request_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return -1;
}

static int32_t fwup_srvop_handle(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  uint8_t *ep;
  FAR int *event = (FAR int *)&cb_args[0];

  /* Expected unsolicited event
   *    %LWM2MOPEV: <event>[,....
   *    %LWM2MEV: <event>[,....
   */
  
  *event = strtol((const char *)pktbuf, (char **)&ep, 10);
  if ((*event == 0) && (pktbuf == ep))
    {
      return -1;
    }

  return 1;
}

static int32_t fwupdate_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return fwup_srvop_handle(pktbuf, pktsz, cb_args, arglen);
}

static int32_t server_op_notice_hndl(FAR uint8_t *pktbuf, size_t pktsz,
                          FAR void **cb_args, size_t arglen)
{
  return fwup_srvop_handle(pktbuf, pktsz, cb_args, arglen);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name: lwm2mstub_get_handler
 ****************************************************************************/

lwm2mstub_hndl_t lwm2mstub_get_handler(FAR uint8_t **pktbuf, size_t *pktsz,
                                      uint32_t *lcmdid)
{
  char *head_pos;
  struct urc_hdltbl_s *tbl;
  size_t shift_size = 0;

  *lcmdid = 0;
  tbl = urc_idhandles;

  while (tbl->head)
    {
      head_pos = strstr((char *)*pktbuf, tbl->head);
      if (head_pos)
        {
          shift_size = head_pos - (char *)*pktbuf + strlen(tbl->head);

          /* Follow shift_size to advance them */

          *pktbuf += shift_size;
          *pktsz -= shift_size;
  
          *lcmdid = tbl->lcmdid;
          return tbl->hdlr;
        }
      tbl++;
    }

  return NULL;
}
