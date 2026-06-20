/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "fh_compression.h"
#include "assertions.h"
#include <string.h>

#define FH_SC_PER_PRB   12
#define FH_VALS_PER_PRB (2 * FH_SC_PER_PRB)

/* ---- bit-stream helpers ---- */

static void pack_bits(uint8_t *stream, int offset, int32_t val, int width)
{
  uint32_t bits = (uint32_t)(val & ((1u << width) - 1));
  int pos = offset >> 3;
  int shift = offset & 7;
  stream[pos] |= (uint8_t)(bits << shift);
  if (shift + width > 8)
    stream[pos + 1] |= (uint8_t)(bits >> (8 - shift));
  if (shift + width > 16)
    stream[pos + 2] |= (uint8_t)(bits >> (16 - shift));
}

static int32_t unpack_bits(const uint8_t *stream, int offset, int width)
{
  int pos = offset >> 3;
  int shift = offset & 7;
  uint32_t bits = (uint32_t)stream[pos] >> shift;
  if (shift + width > 8)
    bits |= (uint32_t)stream[pos + 1] << (8 - shift);
  if (shift + width > 16)
    bits |= (uint32_t)stream[pos + 2] << (16 - shift);
  bits &= (1u << width) - 1;
  uint32_t sign = 1u << (width - 1);
  return (bits & sign) ? (int32_t)(bits | ~((1u << width) - 1)) : (int32_t)bits;
}

/* ---- BFP ---- */

static void bfp_compress_prb(const int16_t *src, int8_t *dst, int iq_bits)
{
  uint16_t max_abs = 1;
  for (int i = 0; i < FH_VALS_PER_PRB; i++) {
    uint16_t v = src[i] < 0 ? (uint16_t)(-(int32_t)src[i]) : (uint16_t)src[i];
    if (v > max_abs)
      max_abs = v;
  }
  int bit_width = 32 - __builtin_clz((unsigned int)max_abs);
  int exponent = bit_width - (iq_bits - 1);
  if (exponent < 0)
    exponent = 0;
  dst[0] = (int8_t)(exponent & 0x0F);
  uint8_t *out = (uint8_t *)(dst + 1);
  memset(out, 0, 3 * iq_bits);
  for (int i = 0; i < FH_VALS_PER_PRB; i++)
    pack_bits(out, i * iq_bits, src[i] >> exponent, iq_bits);
}

static void bfp_decompress_prb(const int8_t *src, int16_t *dst, int iq_bits)
{
  int exponent = (int)((uint8_t)src[0] & 0x0F);
  const uint8_t *in = (const uint8_t *)(src + 1);
  for (int i = 0; i < FH_VALS_PER_PRB; i++)
    dst[i] = (int16_t)(unpack_bits(in, i * iq_bits, iq_bits) << exponent);
}

/* ---- BLKSCALE ---- */

static void blkscale_compress_prb(const int16_t *src, int8_t *dst, int iq_bits)
{
  int32_t max_abs = 1;
  for (int i = 0; i < FH_VALS_PER_PRB; i++) {
    int32_t v = src[i] < 0 ? -(int32_t)src[i] : (int32_t)src[i];
    if (v > max_abs)
      max_abs = v;
  }
  int bit_width = 32 - __builtin_clz((uint32_t)max_abs);
  int shift = bit_width - (iq_bits - 1);
  if (shift < 0)
    shift = 0;
  dst[0] = (int8_t)shift;
  uint8_t *out = (uint8_t *)(dst + 1);
  memset(out, 0, 3 * iq_bits);
  const int32_t clip_max = (1 << (iq_bits - 1)) - 1;
  const int32_t clip_min = -(1 << (iq_bits - 1));
  for (int i = 0; i < FH_VALS_PER_PRB; i++) {
    int32_t val = src[i] >> shift;
    if (val > clip_max)
      val = clip_max;
    else if (val < clip_min)
      val = clip_min;
    pack_bits(out, i * iq_bits, val, iq_bits);
  }
}

static void blkscale_decompress_prb(const int8_t *src, int16_t *dst, int iq_bits)
{
  int shift = (int)(uint8_t)src[0];
  const uint8_t *in = (const uint8_t *)(src + 1);
  for (int i = 0; i < FH_VALS_PER_PRB; i++)
    dst[i] = (int16_t)(unpack_bits(in, i * iq_bits, iq_bits) << shift);
}

/* ---- ULAW ---- */

static int16_t ulaw_encode(int16_t s, int iq_bits)
{
  int sign = s < 0 ? -1 : 1;
  int x = s < 0 ? -(int)s : (int)s;
  x = x > 32734 ? 32734 : x;
  x += 33;
  int seg = 0;
  for (int t = x >> 5; t > 1 && seg < 7; t >>= 1)
    seg++;
  int code = (((x >> (seg + 1)) & 0x0F) | (seg << 4)) ^ 0x7F;
  int peak = (1 << (iq_bits - 1)) - 1;
  return (int16_t)(sign * (code * peak / 127));
}

static int16_t ulaw_decode(int16_t s, int iq_bits)
{
  int sign = s < 0 ? -1 : 1;
  int x = s < 0 ? -(int)s : (int)s;
  int peak = (1 << (iq_bits - 1)) - 1;
  if (peak == 0)
    return 0;
  int code = x * 127 / peak;
  code ^= 0x7F;
  int seg = (code >> 4) & 0x07;
  int decoded = (((code & 0x0F) << 1) | 1) << (seg + 2);
  decoded -= 33;
  if (decoded < 0)
    decoded = 0;
  return (int16_t)(sign * decoded);
}

static void ulaw_compress_prb(const int16_t *src, int8_t *dst, int iq_bits)
{
  dst[0] = 0;
  uint8_t *out = (uint8_t *)(dst + 1);
  memset(out, 0, 3 * iq_bits);
  for (int i = 0; i < FH_VALS_PER_PRB; i++)
    pack_bits(out, i * iq_bits, ulaw_encode(src[i], iq_bits), iq_bits);
}

static void ulaw_decompress_prb(const int8_t *src, int16_t *dst, int iq_bits)
{
  const uint8_t *in = (const uint8_t *)(src + 1);
  for (int i = 0; i < FH_VALS_PER_PRB; i++)
    dst[i] = ulaw_decode((int16_t)unpack_bits(in, i * iq_bits, iq_bits), iq_bits);
}

/* ---- public API ---- */

void fh_compress_prbs(fh_comp_method_t method, int iq_bits, int n_prb, const int16_t *src, int8_t *dst)
{
  AssertFatal(method != FH_COMP_NONE, "fh_compress_prbs called with FH_COMP_NONE\n");
  AssertFatal(iq_bits >= 1 && iq_bits <= 16, "iq_bits %d out of range [1..16]\n", iq_bits);
  const int src_stride = FH_VALS_PER_PRB;
  const int dst_stride = FH_COMP_PRB_BYTES(iq_bits);
  for (int p = 0; p < n_prb; p++) {
    switch (method) {
      case FH_COMP_BFP:
        bfp_compress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      case FH_COMP_BLKSCALE:
        blkscale_compress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      case FH_COMP_ULAW:
        ulaw_compress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      default:
        AssertFatal(0, "Unsupported compression method %d\n", method);
    }
  }
}

void fh_decompress_prbs(fh_comp_method_t method, int iq_bits, int n_prb, const int8_t *src, int16_t *dst)
{
  AssertFatal(method != FH_COMP_NONE, "fh_decompress_prbs called with FH_COMP_NONE\n");
  AssertFatal(iq_bits >= 1 && iq_bits <= 16, "iq_bits %d out of range [1..16]\n", iq_bits);
  const int src_stride = FH_COMP_PRB_BYTES(iq_bits);
  const int dst_stride = FH_VALS_PER_PRB;
  for (int p = 0; p < n_prb; p++) {
    switch (method) {
      case FH_COMP_BFP:
        bfp_decompress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      case FH_COMP_BLKSCALE:
        blkscale_decompress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      case FH_COMP_ULAW:
        ulaw_decompress_prb(src + p * src_stride, dst + p * dst_stride, iq_bits);
        break;
      default:
        AssertFatal(0, "Unsupported compression method %d\n", method);
    }
  }
}

void fh_compress_prach(fh_comp_method_t method, int iq_bits, const int16_t *src, int8_t *dst)
{
  const int total_vals = FH_PRACH_NUM_PRBS * FH_VALS_PER_PRB;
  int16_t padded[FH_PRACH_NUM_PRBS * FH_VALS_PER_PRB];
  memcpy(padded, src, 139 * 2 * sizeof(int16_t));
  memset(padded + 139 * 2, 0, (total_vals - 139 * 2) * sizeof(int16_t));
  fh_compress_prbs(method, iq_bits, FH_PRACH_NUM_PRBS, padded, dst);
}
