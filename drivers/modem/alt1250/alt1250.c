/****************************************************************************
 * drivers/modem/alt1250/alt1250.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <assert.h>

#include "altcom_pkt.h"
#include "altcom_hdlr.h"
#include "altmdm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WRITE_OK 0
#define WRITE_NG 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct alt_queue_s
{
  sq_queue_t queue;
  sem_t lock;
};

struct alt1250_dev_s
{
  FAR struct spi_dev_s *spi;
  FAR const struct alt1250_lower_s *lower;
  sem_t refslock;
  uint8_t crefs;
  struct alt_queue_s waitlist;
  struct alt_queue_s replylist;
  uint64_t evtbitmap;
  sem_t evtmaplock;
  sem_t pfdlock;
  FAR struct pollfd *pfd;
  pthread_t recvthread;
  struct alt_evtbuffer_s *evtbuff;
  uint32_t discardcnt;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int alt1250_open(FAR struct file *filep);
static int alt1250_close(FAR struct file *filep);
static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
  size_t len);
static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int alt1250_poll(FAR struct file *filep, struct pollfd *fds,
  bool setup);

parse_handler_t alt1250_additional_parsehdlr(uint16_t, uint8_t);
compose_handler_t alt1250_additional_composehdlr(uint32_t,
    FAR uint8_t *, size_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_alt1250fops =
{
  alt1250_open,  /* open */
  alt1250_close, /* close */
  alt1250_read,  /* read */
  0,             /* write */
  0,             /* seek */
  alt1250_ioctl, /* ioctl */
  alt1250_poll,  /* poll */
};
static uint8_t g_recvbuff[ALTCOM_PKT_SIZE_MAX];
static uint8_t g_sendbuff[ALTCOM_PKT_SIZE_MAX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_list
 ****************************************************************************/

static void add_list(FAR struct alt_queue_s *head,
  FAR struct alt_container_s *list)
{
  nxsem_wait_uninterruptible(&head->lock);

  sq_addlast(&list->node, &head->queue);

  nxsem_post(&head->lock);
}

/****************************************************************************
 * Name: remove_list_all
 ****************************************************************************/

static FAR struct alt_container_s *remove_list_all(
  FAR struct alt_queue_s *head)
{
  FAR struct alt_container_s *list;

  nxsem_wait_uninterruptible(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  sq_init(&head->queue);

  nxsem_post(&head->lock);

  return list;
}

/****************************************************************************
 * Name: delete_list
 ****************************************************************************/

static FAR struct alt_container_s *delete_list(FAR struct alt_queue_s *head,
  uint16_t cmdid, uint16_t transid)
{
  FAR struct alt_container_s *list;

  nxsem_wait_uninterruptible(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  while (list != NULL)
    {
      if ((list->altcid == cmdid) && (list->alttid == transid))
        {
          sq_rem(&list->node, &head->queue);
          break;
        }

      list = (FAR struct alt_container_s *)sq_next(&list->node);
    }

  nxsem_post(&head->lock);

  return list;
}

/****************************************************************************
 * Name: read_evtbitmap
 ****************************************************************************/

static ssize_t read_evtbitmap(FAR struct alt1250_dev_s *priv,
  FAR char *buffer, size_t len)
{
  int idx;

  DEBUGASSERT(len >= sizeof(priv->evtbitmap));

  nxsem_wait_uninterruptible(&priv->evtmaplock);

  /* change status to NOT WRITABLE */

  for (idx = 0; idx < (sizeof(uint64_t) * 8); idx++)
    {
      if (priv->evtbitmap & (1ULL << idx))
        {
          if (priv->evtbuff->ninst >= idx)
            {
              FAR alt_evtbuf_inst_t *inst = &priv->evtbuff->inst[idx];

              nxsem_wait_uninterruptible(&inst->stat_lock);

              inst->stat = ALTEVTBUF_ST_NOTWRITABLE;

              nxsem_post(&inst->stat_lock);
            }
        }
    }

  memcpy(buffer, &priv->evtbitmap, sizeof(priv->evtbitmap));
  priv->evtbitmap = 0ULL;

  nxsem_post(&priv->evtmaplock);

  return sizeof(priv->evtbitmap);
}

/****************************************************************************
 * Name: write_evtbitmap
 ****************************************************************************/

static void write_evtbitmap(FAR struct alt1250_dev_s *priv,
  uint64_t bitmap)
{
  nxsem_wait_uninterruptible(&priv->evtmaplock);

  priv->evtbitmap |= bitmap;

  nxsem_post(&priv->evtmaplock);
}

/****************************************************************************
 * Name: is_evtbitmap_avail
 ****************************************************************************/

static uint64_t is_evtbitmap_avail(FAR struct alt1250_dev_s *priv)
{
  uint64_t ret;

  nxsem_wait_uninterruptible(&priv->evtmaplock);

  /* 0 means it is not available, otherwise it is available. */

  ret = priv->evtbitmap;

  nxsem_post(&priv->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: add_evtbuff
 ****************************************************************************/

static void add_evtbuff(FAR struct alt1250_dev_s *priv,
  FAR struct alt_evtbuffer_s *buff)
{
  priv->evtbuff = buff;
}

/****************************************************************************
 * Name: write_evtbuff_byidx
 ****************************************************************************/

static int write_evtbuff_byidx(FAR struct alt1250_dev_s *priv,
  uint64_t idx, void(*write_func)(FAR void *outp[], FAR void *inp),
  FAR void *inp)
{
  int ret = WRITE_NG;

  nxsem_wait_uninterruptible(&priv->evtmaplock);

  if (priv->evtbuff)
    {
      if (priv->evtbuff->ninst >= idx)
        {
          FAR alt_evtbuf_inst_t *inst = &priv->evtbuff->inst[idx];

          nxsem_wait_uninterruptible(&inst->stat_lock);
          if (inst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              write_func(inst->outparam, inp);
              priv->evtbitmap |= (1ULL << idx);
              ret = WRITE_OK;
            }

          nxsem_post(&inst->stat_lock);
        }
    }

  nxsem_post(&priv->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: get_evtbuffinst
 ****************************************************************************/

static FAR alt_evtbuf_inst_t *get_evtbuffinst(
  FAR struct alt1250_dev_s *priv, uint16_t cid, uint8_t altver)
{
  FAR alt_evtbuf_inst_t *inst = NULL;
  FAR alt_evtbuf_inst_t *ret = NULL;
  unsigned int i;
  uint16_t cidv1;

  cid &= ~ALTCOM_CMDID_REPLY_BIT;
  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1
       * Even if it cannot be converted, try to search the table
       * using the original command ID.
       */

      cidv1 = convert_cid2v1(cid);
      if (cidv1 != APICMDID_UNKNOWN)
        {
          cid = cidv1;
        }
    }

  nxsem_wait_uninterruptible(&priv->evtmaplock);

  if (priv->evtbuff)
    {
      for (i = 0; i < priv->evtbuff->ninst; i++)
        {
          inst = &priv->evtbuff->inst[i];

          if (inst->altcid == cid)
            {
              nxsem_wait_uninterruptible(&inst->stat_lock);
              if (inst->stat == ALTEVTBUF_ST_WRITABLE)
                {
                  priv->evtbitmap |= (1ULL << i);
                  ret = inst;
                }
              else
                {
                  nxsem_post(&inst->stat_lock);
                }

              break;
            }
        }
    }

  if (!ret)
    {
      nxsem_post(&priv->evtmaplock);
    }

  return ret;
}

/****************************************************************************
 * Name: rel_evtbufinst
 ****************************************************************************/

static void rel_evtbufinst(FAR struct alt1250_dev_s *priv,
  FAR alt_evtbuf_inst_t *inst)
{
  nxsem_post(&inst->stat_lock);
  nxsem_post(&priv->evtmaplock);
}

/****************************************************************************
 * Name: get_evtbuffidx
 ****************************************************************************/

static int get_evtbuffidx(FAR struct alt1250_dev_s *priv, uint16_t cid,
  uint8_t altver)
{
  int i;
  int idx = -1;
  uint16_t cidv1;

  cid &= ~ALTCOM_CMDID_REPLY_BIT;
  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1
       * Even if it cannot be converted, try to search the table
       * using the original command ID.
       */

      cidv1 = convert_cid2v1(cid);
      if (cidv1 != APICMDID_UNKNOWN)
        {
          cid = cidv1;
        }
    }

  if (priv->evtbuff)
    {
      for (i = 0; i < priv->evtbuff->ninst; i++)
        {
          if (priv->evtbuff->inst[i].altcid == cid)
            {
              idx = i;
            }
        }
    }

  if (idx == -1)
    {
      m_warn("event buffer index is not found. cid: 0x%04x\n", cid);
    }

  return idx;
}

/****************************************************************************
 * Name: write_restart_param
 ****************************************************************************/

static void write_restart_param(FAR void *outp[], FAR void *buff)
{
  FAR int *out_reason = (FAR int *)outp[0];
  FAR int *in_reason = (FAR int *)buff;

  *out_reason = *in_reason;
}

/****************************************************************************
 * Name: pollnotify
 ****************************************************************************/

static void pollnotify(FAR struct alt1250_dev_s *priv)
{
  nxsem_wait_uninterruptible(&priv->pfdlock);

  if (priv->pfd)
    {
      /* If poll() waits, notify  */

      priv->pfd->revents |= POLLIN;
      nxsem_post(priv->pfd->sem);
    }

  nxsem_post(&priv->pfdlock);
}

/****************************************************************************
 * Name: get_composehdlr
 ****************************************************************************/

compose_handler_t get_composehdlr(uint32_t cmdid, FAR uint8_t *payload,
  size_t size)
{
  compose_handler_t ret = NULL;

  ret = alt1250_composehdlr(cmdid);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_composehdlr(cmdid, payload, size);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: get_parsehdlr
 ****************************************************************************/

parse_handler_t get_parsehdlr(uint16_t altcid, uint8_t altver)
{
  parse_handler_t ret = NULL;

  ret = alt1250_parsehdlr(altcid, altver);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_parsehdlr(altcid, altver);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: ioctl_power
 ****************************************************************************/

static int ioctl_power(FAR struct alt1250_dev_s *priv,
  FAR struct alt_power_s *req)
{
  int ret = OK;

  switch (req->cmdid)
    {
      case LTE_CMDID_POWERON:
        ret = altmdm_poweron();
        break;

      case LTE_CMDID_POWEROFF:
        ret = altmdm_poweroff();
        break;

      case LTE_CMDID_TAKEWLOCK:
        ret = altmdm_take_wlock();
        break;

      case LTE_CMDID_GIVEWLOCK:
        ret = altmdm_give_wlock();
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ioctl_send
 ****************************************************************************/

static int ioctl_send(FAR struct alt1250_dev_s *priv,
  FAR alt_container_t *req)
{
  int ret = OK;
  compose_handler_t handler;
  uint8_t altver;
  uint16_t cid;
  uint16_t tid;
  FAR uint8_t *payload;

  m_info("send request: command ID=0x%08lx\n", req->cmdid);

  payload = get_payload((FAR struct altcom_cmdhdr_s *)g_sendbuff);

  handler = get_composehdlr(req->cmdid & ~LTE_CMDOPT_ASYNC_BIT, payload,
    ALTCOM_PAYLOAD_SIZE_MAX);
  if (handler)
    {
      altver = altmdm_get_protoversion();
      if (altver == ALTCOM_VERX)
        {
          ret = -ENETDOWN;
        }
      else
        {
          ret = handler(req->inparam, req->inparamlen, altver, payload,
            ALTCOM_PAYLOAD_SIZE_MAX, &cid);
          if (ret >= 0)
            {
              tid = altcom_make_header(
                (FAR struct altcom_cmdhdr_s *)g_sendbuff,
                altver, cid, ret);

              req->altcid = cid | ALTCOM_CMDID_REPLY_BIT;
              req->alttid = tid;

              if (req->outparam != NULL)
                {
                  add_list(&priv->waitlist, req);
                }

              ret = altmdm_write(g_sendbuff, get_pktlen(
                      altver, (uint16_t)ret));
              if (ret < 0)
                {
                  m_err("altmdm_write() failed: %d\n", ret);
                  ret = -ENETDOWN;
                  if (req->outparam != NULL)
                    {
                      delete_list(&priv->waitlist, req->altcid, req->alttid);
                    }
                }
              else
                {
                  m_info("write success: size=%d, cid=0x%04x tid=0x%04x\n",
                    ret, cid, tid);
                  ret = OK;
                }
            }
          else
            {
              m_err("handler() failed: %d\n", ret);
            }
        }
    }
  else
    {
      ret = -ENOSYS;
    }

  return ret;
}

/****************************************************************************
 * Name: altcom_recvthread
 ****************************************************************************/

static void altcom_recvthread(FAR void *arg)
{
  int ret;
  FAR struct alt1250_dev_s *priv = (FAR struct alt1250_dev_s *)arg;
  bool is_running = true;
  FAR struct alt_container_s *head;
  FAR struct alt_container_s *container;
  uint16_t cid;
  uint16_t tid;
  uint8_t altver;
  parse_handler_t handler;

  m_info("recv thread start\n");

  altmdm_init(priv->spi, priv->lower);

  while (is_running)
    {
      ret = altmdm_read(g_recvbuff, ALTCOM_PKT_SIZE_MAX);

      /* Normal packet received */

      if (ret >= 0)
        {
          m_info("read packet %d bytes\n", ret);

          if (altcom_is_pkt_ok(g_recvbuff, ret) < 0)
            {
              /* Forced reset of modem due to packet format error detected */

              m_err("Forced modem reset due to parse failure\n");

              altmdm_reset();
            }
          else
            {
              bool is_discard = false;

              /* parse ALTCOM command ID and transaction ID from header */

              cid = parse_cid((FAR struct altcom_cmdhdr_s *)g_recvbuff);
              tid = parse_tid((FAR struct altcom_cmdhdr_s *)g_recvbuff);
              altver = get_altver(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);

              m_info("receive cid:0x%04x tid:0x%04x\n", cid, tid);

              /* Is error indication packet?
               * This packet is a response to a command that is not supported
               * by the ALT1250. The header of the request packet is included
               * in the contents of the this packet.
               */

              if (is_errind(cid))
                {
                  /* Get ALTCOM command ID and transaction ID
                   * from error indication packet
                   */

                  cid = parse_cid4errind(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);
                  tid = parse_tid4errind(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);

                  m_info("receive errind cid:0x%04x tid:0x%04x\n", cid, tid);

                  container = delete_list(&priv->waitlist, cid, tid);
                  if (container)
                    {
                      /* It means that requested command not implemented
                       * by modem
                       */

                      container->result = -ENOSYS;
                    }
                  else
                    {
                      /* Discard the event packet */

                      is_discard = true;

                      m_warn("container is not found\n");
                    }
                }
              else
                {
                  container = delete_list(&priv->waitlist, cid, tid);

                  handler = get_parsehdlr(cid, altver);
                  if (handler)
                    {
                      FAR uint8_t *payload = get_payload(
                        (FAR struct altcom_cmdhdr_s *)g_recvbuff);

                      if (container)
                        {
                          m_info("handler and container is found\n");

                          /* Perform parse handler */

                          ret = handler(payload, get_payload_len(
                            (FAR struct altcom_cmdhdr_s *)g_recvbuff),
                            altver, container->outparam,
                            container->outparamlen);
                          if (ret < 0)
                            {
                              container->result = ret;
                            }
                        }
                      else
                        {
                          FAR alt_evtbuf_inst_t *inst;

                          m_warn("container is not found\n");

                          inst = get_evtbuffinst(priv, cid, altver);
                          if (inst)
                            {
                              /* Perform parse handler */

                              handler(payload, get_payload_len(
                                (FAR struct altcom_cmdhdr_s *)g_recvbuff),
                                altver, inst->outparam, inst->outparamlen);

                              rel_evtbufinst(priv, inst);
                            }
                          else
                            {
                              /* Discard the event packet */

                              is_discard = true;
                            }
                        }
                    }
                  else if (container)
                    {
                      container->result = -ENOSYS;
                      m_warn("handler is not found\n");
                    }
                  else
                    {
                      /* Discard the event packet */

                      is_discard = true;

                      m_warn("container and handler is not found\n");
                    }
                }

              if (container)
                {
                  add_list(&priv->replylist, container);
                  write_evtbitmap(priv, ALT1250_EVTBIT_REPLY);
                }

              if ((!container) || (container->cmdid & LTE_CMDOPT_ASYNC_BIT))
                {
                  int idx = -1;

                  idx = get_evtbuffidx(priv, cid, altver);
                  if (idx != -1)
                    {
                      write_evtbitmap(priv, 1ULL << idx);
                    }
                }

              if (is_discard)
                {
                  priv->discardcnt++;
                  m_err("discard event %lu\n", priv->discardcnt);
                }
              else
                {
                  pollnotify(priv);
                }
            }
        }
      else
        {
          m_info("read event %d\n", ret);

          switch (ret)
            {
              case ALTMDM_RETURN_RESET_PKT:

                /* If there is a waiting list,
                 * replace it with the replay list.
                 */

                head = remove_list_all(&priv->waitlist);
                add_list(&priv->replylist, head);

                write_evtbitmap(priv, ALT1250_EVTBIT_RESET);
                pollnotify(priv);
                break;

              case ALTMDM_RETURN_RESET_V1:
              case ALTMDM_RETURN_RESET_V4:
                {
                  uint32_t reason = altmdm_get_reset_reason();
                  ret = write_evtbuff_byidx(priv, 0, write_restart_param,
                    (FAR void *)&reason);
                  if (ret == WRITE_OK)
                    {
                      pollnotify(priv);
                    }
                }
                break;

              case ALTMDM_RETURN_EXIT:
                is_running = false;
                break;

              default:
                DEBUGASSERT(0);
                break;
            }
        }
    }

  m_info("recv thread end\n");

  pthread_exit(0);
}

/****************************************************************************
 * Name: alt1250_open
 ****************************************************************************/

static int alt1250_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *priv;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  nxsem_wait_uninterruptible(&priv->refslock);

  if (priv->crefs > 0)
    {
      ret = -EPERM;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;

  nxsem_post(&priv->refslock);

  if (ret == OK)
    {
      nxsem_init(&priv->waitlist.lock, 0, 1);
      nxsem_init(&priv->replylist.lock, 0, 1);
      nxsem_init(&priv->evtmaplock, 0, 1);
      nxsem_init(&priv->pfdlock, 0, 1);

      ret = pthread_create(&priv->recvthread, NULL,
        (pthread_startroutine_t)altcom_recvthread,
        (pthread_addr_t)priv);
      if (ret < 0)
        {
          m_err("thread create failed: %d\n", errno);
          ret = -errno;

          nxsem_destroy(&priv->waitlist.lock);
          nxsem_destroy(&priv->replylist.lock);
          nxsem_destroy(&priv->evtmaplock);
          nxsem_destroy(&priv->pfdlock);

          nxsem_wait_uninterruptible(&priv->refslock);
          priv->crefs--;
          nxsem_post(&priv->refslock);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_close
 ****************************************************************************/

static int alt1250_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *priv;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  nxsem_wait_uninterruptible(&priv->refslock);

  if (priv->crefs == 0)
    {
      ret = -EPERM;
    }
  else
    {
      /* Decrement the count of open references on the driver */

      priv->crefs--;
    }

  nxsem_post(&priv->refslock);

  if (ret == OK)
    {
      nxsem_destroy(&priv->waitlist.lock);
      nxsem_destroy(&priv->replylist.lock);
      nxsem_destroy(&priv->evtmaplock);
      nxsem_destroy(&priv->pfdlock);

      altmdm_fin();
      pthread_join(priv->recvthread, NULL);
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_read
 ****************************************************************************/

static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
  size_t len)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *priv;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  return read_evtbitmap(priv, buffer, len);
}

/****************************************************************************
 * Name: alt1250_ioctl
 ****************************************************************************/

static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *priv;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  switch (cmd)
    {
      case ALT1250_IOC_POWER:
        {
          FAR struct alt_power_s *req = (FAR struct alt_power_s *)arg;

          ret = ioctl_power(priv, req);
        }
        break;

      case ALT1250_IOC_SEND:
        {
          FAR alt_container_t *req = (FAR alt_container_t *)arg;

          ret = ioctl_send(priv, req);
        }
        break;

      case ALT1250_IOC_GETREPLY:
        {
          FAR struct alt_container_s **list =
            (FAR struct alt_container_s **)arg;
          *list = remove_list_all(&priv->replylist);
        }
        break;

      case ALT1250_IOC_SETEVTBUFF:
        {
          FAR struct alt_evtbuffer_s *buff =
            (FAR struct alt_evtbuffer_s *)arg;
          add_evtbuff(priv, buff);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_poll
 ****************************************************************************/

static int alt1250_poll(FAR struct file *filep, FAR struct pollfd *fds,
  bool setup)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *priv;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      nxsem_wait_uninterruptible(&priv->pfdlock);

      if (is_evtbitmap_avail(priv))
        {
          fds->revents |= POLLIN;
          nxsem_post(fds->sem);
        }
      else
        {
          priv->pfd = fds;
        }

      nxsem_post(&priv->pfdlock);
    }
  else
    {
      nxsem_wait_uninterruptible(&priv->pfdlock);
      priv->pfd = NULL;
      nxsem_post(&priv->pfdlock);
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *alt1250_register(FAR const char *devpath,
  FAR struct spi_dev_s *dev, FAR const struct alt1250_lower_s *lower)
{
  FAR struct alt1250_dev_s *priv;
  int ret;

  priv = (FAR struct alt1250_dev_s *)
    kmm_malloc(sizeof(struct alt1250_dev_s));
  if (!priv)
    {
      m_err("Failed to allocate instance.\n");
      return NULL;
    }

  memset(priv, 0, sizeof(struct alt1250_dev_s));

  priv->spi = dev;
  priv->lower = lower;

  nxsem_init(&priv->refslock, 0, 1);

  ret = register_driver(devpath, &g_alt1250fops, 0666, priv);
  if (ret < 0)
    {
      m_err("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return (FAR void *)priv;
}
