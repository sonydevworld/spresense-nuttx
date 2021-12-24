/****************************************************************************
 * include/nuttx/wireless/lte/lte_lwm2m.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LTE_LTE_LWM2M_H
#define __INCLUDE_NUTTX_WIRELESS_LTE_LTE_LWM2M_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LWM2MSTUB_MAX_WRITE_SIZE (1500)
#define LWM2MSTUB_MAX_TOKEN_SIZE (9)

#define LWM2MSTUB_CONDVALID_MINPERIOD  (1<<0)
#define LWM2MSTUB_CONDVALID_MAXPERIOD  (1<<1)
#define LWM2MSTUB_CONDVALID_GRATERTHAN (1<<2)
#define LWM2MSTUB_CONDVALID_LESSTHAN   (1<<3)
#define LWM2MSTUB_CONDVALID_STEP       (1<<4)

#define  LWM2MSTUB_PEND_DL  (0)
#define  LWM2MSTUB_PEND_UPD (1)
#define  LWM2MSTUB_COMP_DL  (2)
#define  LWM2MSTUB_FAIL_DL  (3)
#define  LWM2MSTUB_CANCELED (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lwm2mstub_resource_s
{
  int32_t res_id;
  int32_t res_type;
  int32_t operation;
  int32_t inst_type;
};

struct lwm2mstub_object_s
{
  int object_id;
  int res_num;
  struct lwm2mstub_resource_s rsucs;
};

struct lwm2mstub_instance_s
{
  int32_t object_id;
  int32_t object_inst;
  int32_t res_id;
  int32_t res_inst;
};

struct lwm2mstub_ovcondition_s
{
    uint8_t condValidMask;
    uint32_t minPeriod;
    uint32_t maxPeriod;
    double gt_cond;
    double lt_cond;
    double step_val;
};

typedef void (*lwm2mstub_write_cb_t)(int32_t seq_no, int32_t srv_id,
              struct lwm2mstub_instance_s *inst, char *value, int len);

typedef void (*lwm2mstub_read_cb_t)(int32_t seq_no, int32_t srv_id,
              struct lwm2mstub_instance_s *inst);

typedef void (*lwm2mstub_exec_cb_t)(int32_t seq_no, int32_t srv_id,
              struct lwm2mstub_instance_s *inst, int param);

typedef void (*lwm2mstub_ovstart_cb_t)(int32_t seq_no, int32_t srv_id,
              struct lwm2mstub_instance_s *inst, char *token,
              struct lwm2mstub_ovcondition_s *cond);

typedef void (*lwm2mstub_ovstop_cb_t)(int32_t seq_no, int32_t srv_id,
              struct lwm2mstub_instance_s *inst, char *token);

typedef void (*lwm2mstub_serverop_cb_t)(int event);

typedef void (*lwm2mstub_fwupstate_cb_t)(int event);

#endif  /* __INCLUDE_NUTTX_WIRELESS_LTE_LTE_LWM2M_H */
