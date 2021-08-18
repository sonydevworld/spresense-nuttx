/****************************************************************************
 * drivers/modem/alt1250/altcom_cmd_sock.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H__
#define __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/modem/alt1250.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_NAME_LENGTH        (256)
#define APICMD_SERVNAME_LENGTH    (32)
#define APICMD_H_ADDR_LENGTH      (16)
#define APICMD_OPTVAL_LENGTH      (16)
#define APICMD_DATA_LENGTH        (1500)
#define APICMD_AI_COUNT           (2)

#define APICMD_SELECT_NONBLOCK    (0)
#define APICMD_SELECT_BLOCK       (1)
#define APICMD_SELECT_BLOCKCANCEL (2)
#define APICMD_SELECT_READ_BIT    (1 << 0)
#define APICMD_SELECT_WRITE_BIT   (1 << 1)
#define APICMD_SELECT_EXCEPT_BIT  (1 << 2)

/* Using for socket: Address family */

#define ALTCOM_AF_UNSPEC           0
#define ALTCOM_AF_INET             2
#define ALTCOM_AF_INET6            10
#define ALTCOM_PF_INET             ALTCOM_AF_INET
#define ALTCOM_PF_INET6            ALTCOM_AF_INET6
#define ALTCOM_PF_UNSPEC           ALTCOM_AF_UNSPEC

/* Using for socket: Socket protocol type */

#define ALTCOM_SOCK_STREAM         1
#define ALTCOM_SOCK_DGRAM          2
#define ALTCOM_SOCK_RAW            3
#define ALTCOM_SOCK_DGRAM_DTLS     130

/* Using for socket: Protocol */

#define ALTCOM_IPPROTO_IP          0
#define ALTCOM_IPPROTO_ICMP        1
#define ALTCOM_IPPROTO_TCP         6
#define ALTCOM_IPPROTO_UDP         17
#define ALTCOM_IPPROTO_IPV6        41
#define ALTCOM_IPPROTO_ICMPV6      58
#define ALTCOM_IPPROTO_UDPLITE     136
#define ALTCOM_IPPROTO_RAW         255

/* Using for recvfrom/sendto: Flags */

#define ALTCOM_MSG_PEEK            0x01
#define ALTCOM_MSG_WAITALL         0x02
#define ALTCOM_MSG_OOB             0x04
#define ALTCOM_MSG_DONTWAIT        0x08
#define ALTCOM_MSG_MORE            0x10

/* Using for setsockopt/getsockopt: Level */

#define ALTCOM_SOL_SOCKET          0xfff

/* Using for setsockopt/getsockopt: Option flags per-socket */

#define ALTCOM_SO_REUSEADDR        0x0004
#define ALTCOM_SO_KEEPALIVE        0x0008
#define ALTCOM_SO_BROADCAST        0x0020

/* Using for setsockopt/getsockopt:
 * Additional options, not kept in so_options
 */

#define ALTCOM_SO_ACCEPTCONN       0x0002
#define ALTCOM_SO_LINGER           0x0080
#define ALTCOM_SO_RCVBUF           0x1002
#define ALTCOM_SO_SNDTIMEO         0x1005
#define ALTCOM_SO_RCVTIMEO         0x1006
#define ALTCOM_SO_ERROR            0x1007
#define ALTCOM_SO_TYPE             0x1008
#define ALTCOM_SO_NO_CHECK         0x100a

/****************************************************************************
 * Public Types
 ****************************************************************************/


typedef uint8_t  altcom_sa_family_t;
typedef uint32_t altcom_socklen_t;
typedef uint16_t altcom_in_port_t;

typedef uint32_t altcom_in_addr_t;

struct altcom_in_addr
{
  altcom_in_addr_t s_addr;
};

struct altcom_in6_addr
{
  union
  {
    uint32_t u32_addr[4];
    uint16_t u16_addr[8];
    uint8_t  u8_addr[16];
  } un;
#define altcom_s6_addr  un.u8_addr
};

struct altcom_sockaddr_storage
{
  uint8_t            s2_len;
  altcom_sa_family_t ss_family;
  char               s2_data1[2];
  uint32_t           s2_data2[3];
  uint32_t           s2_data3[3];
};

struct altcom_sockaddr_in
{
  uint8_t                sin_len;
  altcom_sa_family_t     sin_family;
  altcom_in_port_t       sin_port;
  struct altcom_in_addr  sin_addr;
#define ALTCOM_SIN_ZERO_LEN 8
  char                   sin_zero[ALTCOM_SIN_ZERO_LEN];
};

struct altcom_sockaddr_in6
{
  uint8_t                sin6_len;
  altcom_sa_family_t     sin6_family;
  altcom_in_port_t       sin6_port;
  uint32_t               sin6_flowinfo;
  struct altcom_in6_addr sin6_addr;
  uint32_t               sin6_scope_id;
};

/* structure for APICMDID_SOCK_ACCEPT and APICMDID_SOCK_GETSOCKNAME */

begin_packed_struct struct altmdmpkt_sockaddrlen_s
{
  int32_t sockfd;
  uint32_t addrlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_BIND and APICMDID_SOCK_CONNECT */

begin_packed_struct struct altmdmpkt_sockaddr_s
{
  int32_t sockfd;
  uint32_t namelen;
  struct altcom_sockaddr_storage name;
} end_packed_struct;

/* structure for APICMDID_SOCK_CLOSE */

begin_packed_struct struct apicmd_close_s
{
  int32_t sockfd;
} end_packed_struct;

/* structure for APICMDID_SOCK_FCNTL */

begin_packed_struct struct apicmd_fcntl_s
{
  int32_t sockfd;
  int32_t cmd;
  int32_t val;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfo_s
{
  uint32_t nodenamelen;
  int8_t nodename[APICMD_NAME_LENGTH];
  uint32_t servnamelen;
  int8_t servname[APICMD_SERVNAME_LENGTH];
  int32_t hints_flag;
  int32_t ai_flags;
  int32_t ai_family;
  int32_t ai_socktype;
  int32_t ai_protocol;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAME */

begin_packed_struct struct apicmd_gethostbyname_s
{
  uint32_t namelen;
  int8_t name[APICMD_NAME_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAMER */

begin_packed_struct struct apicmd_gethostbynamer_s
{
  uint32_t namelen;
  int8_t name[APICMD_NAME_LENGTH];
  int32_t buflen;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETSOCKOPT */

begin_packed_struct struct apicmd_getsockopt_s
{
  int32_t sockfd;
  int32_t level;
  int32_t optname;
  int32_t optlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_LISTEN */

begin_packed_struct struct apicmd_listen_s
{
  int32_t sockfd;
  int32_t backlog;
} end_packed_struct;

/* structure for APICMDID_SOCK_RECV */

begin_packed_struct struct apicmd_recv_s
{
  int32_t sockfd;
  int32_t recvlen;
  int32_t flags;
} end_packed_struct;

/* structure for APICMDID_SOCK_RECVFROM */

begin_packed_struct struct apicmd_recvfrom_s
{
  int32_t sockfd;
  int32_t recvlen;
  int32_t flags;
  uint32_t fromlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_SELECT */

begin_packed_struct struct apicmd_select_s
{
  int32_t request;
  int32_t id;
  int32_t maxfds;
  uint16_t used_setbit;
  altcom_fd_set readset;
  altcom_fd_set writeset;
  altcom_fd_set exceptset;
} end_packed_struct;

/* structure for APICMDID_SOCK_SEND */

begin_packed_struct struct apicmd_send_s
{
  int32_t sockfd;
  int32_t flags;
  int32_t datalen;
  int8_t senddata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SENDTO */

begin_packed_struct struct apicmd_sendto_s
{
  int32_t sockfd;
  int32_t flags;
  int32_t datalen;
  uint32_t tolen;
  struct altcom_sockaddr_storage to;
  int8_t senddata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SETSOCKOPT */

begin_packed_struct struct apicmd_setsockopt_s
{
  int32_t sockfd;
  int32_t level;
  int32_t optname;
  int32_t optlen;
  int8_t optval[APICMD_OPTVAL_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SHUTDOWN */

begin_packed_struct struct apicmd_shutdown_s
{
  int32_t sockfd;
  int32_t how;
} end_packed_struct;

/* structure for APICMDID_SOCK_SOCKET */

begin_packed_struct struct apicmd_socket_s
{
  int32_t domain;
  int32_t type;
  int32_t protocol;
} end_packed_struct;

/****************************************************************************/

/* structure for APICMDID_SOCK_BIND and APICMDID_SOCK_CLOSE,
 * APICMDID_SOCK_CONNECT, APICMDID_SOCK_FCNTL, APICMDID_SOCK_LISTEN,
 * APICMDID_SOCK_SEND, APICMDID_SOCK_SENDTO, APICMDID_SOCK_SETSOCKOPT,
 * APICMDID_SOCK_SHUTDOWN, APICMDID_SOCK_SOCKET
 */

begin_packed_struct struct altmdmpktr_sockcomm_s
{
  int32_t ret_code;
  int32_t err_code;
} end_packed_struct;

/* structure for APICMDID_SOCK_ACCEPT and APICMDID_SOCK_GETSOCKNAME */

begin_packed_struct struct altmdmpktr_sockaddr_s
{
  int32_t ret_code;
  int32_t err_code;
  uint32_t addrlen;
  struct altcom_sockaddr_storage address;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfo_ai_s
{
  int32_t ai_flags;
  int32_t ai_family;
  int32_t ai_socktype;
  int32_t ai_protocol;
  altcom_socklen_t ai_addrlen;
  struct altcom_sockaddr_storage ai_addr;
  uint32_t ai_cnamelen;
  int8_t ai_canonname[APICMD_NAME_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfores_s
{
  int32_t ret_code;
  uint32_t ai_num;
  struct apicmd_getaddrinfo_ai_s ai[APICMD_AI_COUNT];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAME */

begin_packed_struct struct apicmd_gethostbynameres_s
{
  int32_t ret_code;
  int8_t h_name[APICMD_NAME_LENGTH];
  int8_t h_aliases[APICMD_NAME_LENGTH];
  int32_t h_addrtype;
  int32_t h_length;
  int8_t h_addr_list[APICMD_H_ADDR_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAMER */

begin_packed_struct struct apicmd_gethostbynamer_res_s
{
  int32_t ret_code;
  int32_t err_code;
  int8_t h_name[APICMD_NAME_LENGTH];
  int8_t h_aliases[APICMD_NAME_LENGTH];
  int32_t h_addrtype;
  int32_t h_length;
  int8_t h_addr_list[APICMD_H_ADDR_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETSOCKOPT */

begin_packed_struct struct apicmd_getsockoptres_s
{
  int32_t ret_code;
  int32_t err_code;
  int32_t optlen;
  int8_t optval[APICMD_OPTVAL_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_RECV */

begin_packed_struct struct apicmd_recvres_s
{
  int32_t ret_code;
  int32_t err_code;
  int8_t recvdata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_RECVFROM */

begin_packed_struct struct apicmd_recvfromres_s
{
  int32_t ret_code;
  int32_t err_code;
  uint32_t fromlen;
  struct altcom_sockaddr_storage from;
  int8_t recvdata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SELECT */

begin_packed_struct struct apicmd_selectres_s
{
  int32_t ret_code;
  int32_t err_code;
  int32_t id;
  uint16_t used_setbit;
  altcom_fd_set readset;
  altcom_fd_set writeset;
  altcom_fd_set exceptset;
} end_packed_struct;

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H__ */
