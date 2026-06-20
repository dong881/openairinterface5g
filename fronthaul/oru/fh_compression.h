/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FH_COMP_NONE     = 0,
  FH_COMP_BFP      = 1,
  FH_COMP_BLKSCALE = 2,
  FH_COMP_ULAW     = 3,
} fh_comp_method_t;

#define FH_COMP_NUM_METHODS 4

/* Compressed byte budget per PRB: 1 header byte + 3 × iq_bits packed sample bytes.
 * Applies to BFP, BLKSCALE, and ULAW. Typical iq_bits values: 8, 9, 12, 14. */
#define FH_COMP_PRB_BYTES(iq_bits) (1 + 3 * (iq_bits))

/* PRACH carries 139 subcarriers, zero-padded to this many PRBs before compression. */
#define FH_PRACH_NUM_PRBS 12

void fh_compress_prbs(fh_comp_method_t method, int iq_bits, int n_prb, const int16_t *src, int8_t *dst);
void fh_decompress_prbs(fh_comp_method_t method, int iq_bits, int n_prb, const int8_t *src, int16_t *dst);
void fh_compress_prach(fh_comp_method_t method, int iq_bits, const int16_t *src, int8_t *dst);

#ifdef __cplusplus
}
#endif
