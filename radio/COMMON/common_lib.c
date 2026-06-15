/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief common APIs for different RF frontend device
 */
#include <pthread.h>
#include <stdio.h>
#include <strings.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "common_lib.h"
#include "assertions.h"
#include "common/utils/load_module_shlib.h"
#include "common/utils/LOG/log.h"
#include "executables/softmodem-common.h"
#include "common/config/config_paramdesc.h"
#include "common/config/config_userapi.h"
#include "common/cmake_defs.h"
#include "openair1/PHY/TOOLS/tools_defs.h"

#define MAX_GAP 100ULL
const char *const devtype_names[MAX_RF_DEV_TYPE] =
    {"", "USRP B200", "USRP X300", "USRP N300", "USRP X400", "BLADERF", "LMSSDR", "IRIS", "No HW", "UEDv2", "RFSIMULATOR"};

const char *get_devname(int devtype) {
  if (devtype < MAX_RF_DEV_TYPE && devtype !=MIN_RF_DEV_TYPE )
    return devtype_names[devtype];
  return "none";
}

static int set_device(openair0_device_t *device)
{
  char *dev_type = device->host_type == RAU_HOST ? "RAU" : "RRU";
  const char *devname = get_devname(device->type);
  if (strcmp(devname, "none") != 0) {
    LOG_I(HW, "[%s] has loaded %s device.\n", dev_type, devname);
    return 0;
  }
  LOG_E(HW, "[%s] invalid HW device.\n", dev_type);
  return -1;
}

static int set_transport(openair0_device_t *device)
{
  char *dev_type = device->host_type == RAU_HOST ? "RAU" : "RRU";
  switch (device->transp_type) {
    case ETHERNET_TP:
      LOG_I(HW, "[%s] has loaded ETHERNET trasport protocol.\n", dev_type);
      return 0;
      break;

    case NONE_TP:
      LOG_I(HW, "[%s] has not loaded a transport protocol.\n", dev_type);
      return 0;
      break;

    default:
      LOG_E(HW, "[%s] invalid transport protocol.\n", dev_type);
      return -1;
      break;
  }
}

/* look for the interface library and load it */
int load_lib(openair0_device_t *device, openair0_config_t *openair0_cfg, eth_params_t *eth_cfg, rau_type_t rau_type)
{
  openair0_cfg->command_line_sample_advance = get_softmodem_params()->command_line_sample_advance;

  openair0_cfg->recplay_mode = read_recplayconfig(&openair0_cfg->recplay_conf, &device->recplay_state);
  // softmodem has to know we use the iqrecorder to workaround randomized algorithms
  IS_SOFTMODEM_IQRECORDER = openair0_cfg->recplay_mode == RECPLAY_RECORDMODE;

  char *deflibname = OAI_RF_LIBNAME;
  loader_shlibfunc_t shlib_fdesc = {.fname = "device_init"};
  if (openair0_cfg->recplay_mode == RECPLAY_REPLAYMODE) {
    deflibname = OAI_IQPLAYER_LIBNAME;
    IS_SOFTMODEM_IQPLAYER = true; // softmodem has to know we use the iqplayer to workaround randomized algorithms
  } else {
    switch (rau_type) {
      case RAU_LOCAL_RADIO_HEAD:
        if (IS_SOFTMODEM_RFSIM)
          deflibname = OAI_RFSIM_LIBNAME;
        break;
      case RAU_REMOTE_THIRDPARTY_RADIO_HEAD:
        deflibname = OAI_THIRDPARTY_TP_LIBNAME;
        shlib_fdesc.fname = "transport_init";
        break;
      case RAU_REMOTE_RADIO_HEAD:
        deflibname = OAI_TP_LIBNAME;
        shlib_fdesc.fname = "transport_init";
        break;
      default:
        AssertFatal(false, "impossible radio head\n");
    }
  }

  char *devname = NULL;
  paramdef_t device_params = {"name", CONFIG_HLP_DEVICE, 0, .strptr = &devname, .defstrval = deflibname, TYPE_STRING, 0};
  config_get(config_get_if(), &device_params, 1, DEVICE_SECTION);

  int ret = load_module_shlib(devname, &shlib_fdesc, 1, NULL);
  AssertFatal(ret >= 0, "Library %s couldn't be loaded\n", devname);
  typedef int (*devfunc_t)(openair0_device_t *, openair0_config_t *, eth_params_t *);
  return ((devfunc_t)shlib_fdesc.fptr)(device, openair0_cfg, eth_cfg);
}

int openair0_device_load(openair0_device_t *device, openair0_config_t *openair0_cfg)
{
  int rc=0;
  rc=load_lib(device, openair0_cfg, NULL,RAU_LOCAL_RADIO_HEAD );

  if ( rc >= 0) {
    if ( set_device(device) < 0) {
      LOG_E(HW, "%s %d:Unsupported radio head\n", __FILE__, __LINE__);
      return -1;
    }
  } else {
    AssertFatal(false, "can't open the radio device: %s\n", get_devname(device->type));
  }
  pthread_mutex_init(&device->reOrder.mutex_store, NULL);
  pthread_mutex_init(&device->reOrder.mutex_write, NULL);
  return rc;
}

int openair0_transport_load(openair0_device_t *device, openair0_config_t *openair0_cfg, eth_params_t *eth_params)
{
  int rc = load_lib(device, openair0_cfg, eth_params, RAU_REMOTE_RADIO_HEAD);

  if ( rc >= 0) {
    if ( set_transport(device) < 0) {
      LOG_E(HW, "%s %d:Unsupported transport protocol\n", __FILE__, __LINE__);
      return -1;
    }
  }

  return rc;
}

static void
add_new_buff(re_order_t *ctx, openair0_timestamp_t timestamp, void **txp, int nsamps, int nb_writers, int nbAnt, int flags)
{
  struct reorder_queue *i = ctx->queue;
  struct reorder_queue *end = i + WRITE_QUEUE_SZ;
  for (i = ctx->queue; i < end; i++) {
    if (!i->active) {
      LOG_D(HW, "Creating one ue signal for ts %ld, nsamps %d, nb writers %d\n", timestamp, nsamps, nb_writers);
      *i = (struct reorder_queue){.timestamp = timestamp,
                                  .active = true,
                                  .nsamps = nsamps,
                                  .nbAnt = nbAnt,
                                  .flags = flags,
                                  .nb_writers = nb_writers,
                                  .already_wrote = 1};
      i->txp = malloc(nbAnt * sizeof(c16_t *));
      i->txp[0] = malloc(nbAnt * nsamps * sizeof(c16_t));
      memcpy(i->txp[0], txp[0], nsamps * sizeof(c16_t));
      for (int j = 1; j < nbAnt; j++) {
        i->txp[j] = i->txp[0] + nsamps;
        memcpy(i->txp[j], txp[j], nsamps * sizeof(c16_t));
      }
      break;
    }
  }
  AssertFatal(i < end, "Write queue full\n");
}

static void
writerEnqueue(re_order_t *ctx, openair0_timestamp_t timestamp, void **txp, int nsamps, int nb_writers, int nbAnt, int flags)
{
  AssertFatal(nb_writers, "");
  AssertFatal(nbAnt, "");

  pthread_mutex_lock(&ctx->mutex_store);
  LOG_D(HW, "Enqueue write for TS: %lu\n", timestamp);
  struct reorder_queue *i = ctx->queue;
  struct reorder_queue *end = i + WRITE_QUEUE_SZ;
  if (nb_writers > 1) {
    // search existing block
    for (i = ctx->queue; i < end; i++) {
      if (i->active && i->timestamp == timestamp && i->nsamps == nsamps) {
        AssertFatal(i->nbAnt = nbAnt && i->nb_writers == nb_writers, "");
        if (i->nsamps == nsamps && i->timestamp == timestamp) {
          LOG_D(HW, "Adding one ue signal for ts %ld, nsamps %d, %d/%d\n", timestamp, nsamps, i->already_wrote, nb_writers);
          i->already_wrote++;
          for (int j = 0; j < nbAnt; j++) {
            c16_t *buf = (c16_t *)i->txp[j];
            c16_t *in = (c16_t *)txp[j];
            for (int k = 0; k < nsamps; k++)
              buf[k] = c16add(buf[k], in[k]);
          }
        } else {
          AssertFatal(false, "Need to split block\n");
        }
        break;
      }
    }
    if (i == end)
      add_new_buff(ctx, timestamp, txp, nsamps, nb_writers, nbAnt, flags);
  } else {
    for (i = ctx->queue; i < end; i++) {
      if (!i->active) {
        i->timestamp = timestamp;
        i->active = true;
        i->nsamps = nsamps;
        i->nbAnt = nbAnt;
        i->flags = flags;
        AssertFatal(nbAnt <= NB_ANTENNAS_TX, "");
        for (int j = 0; j < nbAnt; j++)
          i->txp[j] = txp[j];
        break;
      }
    }
    AssertFatal(i < end, "Write queue full\n");
  }
  pthread_mutex_unlock(&ctx->mutex_store);
}

static void writerProcessWaitingQueue(nrue_ru_write_t nrue_ru_write, PHY_VARS_NR_UE *UE, openair0_device_t *device)
{
  bool found = false;
  re_order_t *ctx = &device->reOrder;
  do {
    found = false;
    pthread_mutex_lock(&ctx->mutex_store);
    for (struct reorder_queue *i = ctx->queue; i < ctx->queue + WRITE_QUEUE_SZ; i++) {
      if (i->active && i->nb_writers == i->already_wrote && llabs(i->timestamp - ctx->nextTS) < MAX_GAP) {
        openair0_timestamp_t timestamp = i->timestamp;
        LOG_D(HW, "Dequeue write for TS: %lu\n", timestamp);
        int nsamps = i->nsamps;
        int nbAnt = i->nbAnt;
        int flags = i->flags;
        int nb_writers = i->nb_writers;
        void **txpp = i->txp;
        void *txp[NB_ANTENNAS_TX];
        AssertFatal(nbAnt <= NB_ANTENNAS_TX, "");
        for (int j = 0; j < nbAnt; j++) {
          txp[j] = i->txp[j];
          i->txp[j] = NULL;
        }
        i->active = false;
        pthread_mutex_unlock(&ctx->mutex_store);
        found = true;
        LOG_D(HW, "sending for ts %ld, nsamps %d, flags %d, rfsim %d\n", timestamp, nsamps, flags, IS_SOFTMODEM_RFSIM);
        if (flags || IS_SOFTMODEM_RFSIM) {
          int wroteSamples;
          if (nrue_ru_write)
            wroteSamples = nrue_ru_write(UE, timestamp, txp, nsamps, nbAnt, flags);
          else
            wroteSamples = device->trx_write_func(device, timestamp, txp, nsamps, nbAnt, flags);
          if (wroteSamples != nsamps)
            LOG_W(HW, "Failed to write to RF: wrote %d out of %d samples\n", wroteSamples, nsamps);
        }
        ctx->nextTS = timestamp + nsamps;
        if (nb_writers > 1) {
          free(txp[0]);
          free(txpp);
        }
        pthread_mutex_lock(&ctx->mutex_store);
      }
    }
    pthread_mutex_unlock(&ctx->mutex_store);
  } while (found);
}

// mutex (or atomic flags) will be mandatory because this out order system root cause is there are several writer threads
int openair0_write_reorder_common(nrue_ru_write_t nrue_ru_write,
                                  PHY_VARS_NR_UE *UE,
                                  openair0_device_t *device,
                                  openair0_timestamp_t timestamp,
                                  void **txp,
                                  int nsamps,
                                  int nb_writers,
                                  int nbAnt,
                                  int flags)
{
  int wroteSamples = 0;
  re_order_t *ctx = &device->reOrder;
  LOG_D(HW, "received write order ts: %lu, nb samples %d, next ts %luflags %d\n", timestamp, nsamps, timestamp + nsamps, flags);
  pthread_mutex_lock(&ctx->mutex_store);
  if (!ctx->initDone) {
    ctx->nextTS = timestamp;
    for (int i = 0; i < WRITE_QUEUE_SZ; i++) {
      ctx->queue[i].txp = malloc(sizeof(void *) * NB_ANTENNAS_TX);
    }
    ctx->initDone = true;
  }
  pthread_mutex_unlock(&ctx->mutex_store);
  if (pthread_mutex_trylock(&ctx->mutex_write) == 0) {
    // We have the write exclusivity
    AssertFatal(nb_writers, "");
    AssertFatal(nbAnt, "");
    if (llabs(timestamp - ctx->nextTS) < MAX_GAP && nb_writers == 1) { // We are writing in sequence of the previous write
      if (flags || IS_SOFTMODEM_RFSIM) {
        if (nrue_ru_write)
          wroteSamples = nrue_ru_write(UE, timestamp, txp, nsamps, nbAnt, flags);
        else
          wroteSamples = device->trx_write_func(device, timestamp, txp, nsamps, nbAnt, flags);
        if (wroteSamples != nsamps)
          LOG_W(HW, "Failed to write to RF: wrote %d out of %d samples\n", wroteSamples, nsamps);
      } else
        wroteSamples = nsamps;
      ctx->nextTS = timestamp + nsamps;

    } else {
      writerEnqueue(ctx, timestamp, txp, nsamps, nb_writers, nbAnt, flags);
    }
    writerProcessWaitingQueue(nrue_ru_write, UE, device);
    pthread_mutex_unlock(&ctx->mutex_write);
    return wroteSamples ? wroteSamples : nsamps;
  }
  writerEnqueue(ctx, timestamp, txp, nsamps, nb_writers, nbAnt, flags);
  if (pthread_mutex_trylock(&ctx->mutex_write) == 0) {
    writerProcessWaitingQueue(nrue_ru_write, UE, device);
    pthread_mutex_unlock(&ctx->mutex_write);
  }
  return nsamps;
}

int openair0_write_reorder(openair0_device_t *device, openair0_timestamp_t timestamp, void **txp, int nsamps, int nbAnt, int flags)
{
  return openair0_write_reorder_common(NULL, NULL, device, timestamp, txp, nsamps, 1, nbAnt, flags);
}

void openair0_write_reorder_clear_context(openair0_device_t *device)
{
  LOG_W(HW, "received write reorder clear context\n");
  re_order_t *ctx = &device->reOrder;
  if (!ctx->initDone)
    return;
  if (pthread_mutex_trylock(&ctx->mutex_write) != 0)
    LOG_E(HW, "write_reorder_clear_context call while still writing on the device\n");
  else
    pthread_mutex_unlock(&ctx->mutex_write);
  pthread_mutex_lock(&ctx->mutex_store);
  for (int i = 0; i < WRITE_QUEUE_SZ; i++) {
    ctx->queue[i].active = false;
    free(ctx->queue[i].txp);
  }
  ctx->initDone = false;
  pthread_mutex_unlock(&ctx->mutex_store);
}
