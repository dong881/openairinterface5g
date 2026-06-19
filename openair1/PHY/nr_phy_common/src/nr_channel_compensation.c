/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "nr_channel_compensation.h"
#include "bits.h"
#include <complex.h>
#include "PHY/sse_intrin.h"
#include "log.h"
#include "PHY/impl_defs_top.h"
#ifndef __AVX2__
#define USE128BIT
#endif

#ifdef __AVX512BW__
#define s16 __m512i
#define setzero _mm512_setzero_si512
#define set1_s16 _mm512_set1_epi16
#define cpx_mult_conj_s16 oai_mm512_cpx_mult_conj
#define adds_s16 _mm512_adds_epi16
#define smadd_s16 oai_mm512_smadd
#define packs_s16 _mm512_packs_epi32
#define unpacklo_s16 _mm512_unpacklo_epi16
#define mulhrs_s16 _mm512_mulhrs_epi16
#define COMPSHIFT 4
#elif defined(__AVX2__)
#define s16 simde__m256i
#define setzero simde_mm256_setzero_si256
#define set1_s16 simde_mm256_set1_epi16
#define cpx_mult_conj_s16 oai_mm256_cpx_mult_conj
#define adds_s16 simde_mm256_adds_epi16
#define smadd_s16 oai_mm256_smadd
#define packs_s16 simde_mm256_packs_epi32
#define unpacklo_s16 simde_mm256_unpacklo_epi16
#define mulhrs_s16 simde_mm256_mulhrs_epi16
#define COMPSHIFT 3
#else
#define s16 simde__m128i
#define setzero simde_mm_setzero_si128
#define set1_s16 simde_mm_set1_epi16
#define cpx_mult_conj_s16 oai_mm_cpx_mult_conj
#define adds_s16 simde_mm_adds_epi16
#define smadd_s16 oai_mm_smadd
#define packs_s16 simde_mm_packs_epi32
#define unpacklo_s16 simde_mm_unpacklo_epi16
#define mulhrs_s16 simde_mm_mulhrs_epi16
#define COMPSHIFT 2
#endif

#ifndef __aarch64__
void nr_channel_compensation(uint32_t buffer_length,
                             int nb_rx_ant,
                             int nb_layers,
                             c16_t rxFext[nb_rx_ant][buffer_length],
                             c16_t chFext[nb_layers][nb_rx_ant][buffer_length],
                             c16_t ch_maga[nb_layers][buffer_length],
                             c16_t ch_magb[nb_layers][buffer_length],
                             c16_t ch_magc[nb_layers][buffer_length],
                             c16_t **rxComp,
                             c16_t (*rho)[nb_layers][buffer_length],
                             int mod_order,
                             uint32_t symbol,
                             uint32_t output_shift)
{
  s16 QAM_ampa = setzero();
  s16 QAM_ampb = setzero();
  s16 QAM_ampc = setzero();

  if (mod_order == 4) {
    QAM_ampa = set1_s16(QAM16_n1);
  } else if (mod_order == 6) {
    QAM_ampa = set1_s16(QAM64_n1);
    QAM_ampb = set1_s16(QAM64_n2);
  } else if (mod_order == 8) {
    QAM_ampa = set1_s16(QAM256_n1);
    QAM_ampb = set1_s16(QAM256_n2);
    QAM_ampc = set1_s16(QAM256_n3);
  }

  for (int aatx = 0; aatx < nb_layers; aatx++) {
    s16 *rxComp_s16 = (s16 *)&rxComp[aatx][symbol * buffer_length];
    s16 *ch_maga_s16 = (s16 *)ch_maga[aatx];
    s16 *ch_magb_s16 = (s16 *)ch_magb[aatx];
    s16 *ch_magc_s16 = (s16 *)ch_magc[aatx];

    // First Rx antenna: direct store — eliminates need to pre memset the output buffers
    {
      s16 *rxF = (s16 *)rxFext[0];
      s16 *chF = (s16 *)chFext[aatx][0];

      for (uint32_t i = 0; i < buffer_length >> COMPSHIFT; i++) {
        rxComp_s16[i] = cpx_mult_conj_s16(chF[i], rxF[i], output_shift);

        if (mod_order > 2) {
          s16 mag = smadd_s16(chF[i], chF[i], output_shift);
          mag = packs_s16(mag, mag);
          mag = unpacklo_s16(mag, mag);
          ch_maga_s16[i] = mulhrs_s16(mag, QAM_ampa);

          if (mod_order > 4)
            ch_magb_s16[i] = mulhrs_s16(mag, QAM_ampb);

          if (mod_order > 6)
            ch_magc_s16[i] = mulhrs_s16(mag, QAM_ampc);
        }
      }

      if (rho) {
        for (int atx = 0; atx < nb_layers; atx++) {
          s16 *rho_s16 = (s16 *)rho[aatx][atx];
          s16 *chF2 = (s16 *)chFext[atx][0];
          for (uint32_t i = 0; i < buffer_length >> COMPSHIFT; i++)
            rho_s16[i] = cpx_mult_conj_s16(chF[i], chF2[i], output_shift);
        }
      }
    }

    // Remaining Rx antennas: accumulate (MRC)
    for (int aarx = 1; aarx < nb_rx_ant; aarx++) {
      s16 *rxF = (s16 *)rxFext[aarx];
      s16 *chF = (s16 *)chFext[aatx][aarx];

      for (uint32_t i = 0; i < buffer_length >> COMPSHIFT; i++) {
        s16 comp = cpx_mult_conj_s16(chF[i], rxF[i], output_shift);
        rxComp_s16[i] = adds_s16(rxComp_s16[i], comp);

        if (mod_order > 2) {
          s16 mag = smadd_s16(chF[i], chF[i], output_shift);
          mag = packs_s16(mag, mag);
          mag = unpacklo_s16(mag, mag);
          ch_maga_s16[i] = adds_s16(ch_maga_s16[i], mulhrs_s16(mag, QAM_ampa));

          if (mod_order > 4)
            ch_magb_s16[i] = adds_s16(ch_magb_s16[i], mulhrs_s16(mag, QAM_ampb));

          if (mod_order > 6)
            ch_magc_s16[i] = adds_s16(ch_magc_s16[i], mulhrs_s16(mag, QAM_ampc));
        }
      }

      if (rho) {
        for (int atx = 0; atx < nb_layers; atx++) {
          s16 *rho_s16 = (s16 *)rho[aatx][atx];
          s16 *chF2 = (s16 *)chFext[atx][aarx];
          for (uint32_t i = 0; i < buffer_length >> COMPSHIFT; i++)
            rho_s16[i] = adds_s16(rho_s16[i], cpx_mult_conj_s16(chF[i], chF2[i], output_shift));
        }
      }
    }
  }
}

#else
#define COMP_SHIFT(SHIFT)                                                                   \
  static void nr_channel_compensation##SHIFT(uint32_t buffer_length,                        \
                                             int nb_rx_ant,                                 \
                                             int nb_layers,                                 \
                                             c16_t rxFext[][buffer_length],                 \
                                             c16_t chFext[][nb_rx_ant][buffer_length],      \
                                             c16_t ch_maga[][buffer_length],                \
                                             c16_t ch_magb[][buffer_length],                \
                                             c16_t ch_magc[][buffer_length],                \
                                             c16_t **rxComp,                                \
                                             c16_t(*rho)[nb_layers][buffer_length],         \
                                             int mod_order,                                 \
                                             uint32_t symbol)                               \
  {                                                                                         \
    s16 QAM_ampa = setzero();                                                               \
    s16 QAM_ampb = setzero();                                                               \
    s16 QAM_ampc = setzero();                                                               \
                                                                                            \
    if (mod_order == 4) {                                                                   \
      QAM_ampa = set1_s16(QAM16_n1);                                                        \
    } else if (mod_order == 6) {                                                            \
      QAM_ampa = set1_s16(QAM64_n1);                                                        \
      QAM_ampb = set1_s16(QAM64_n2);                                                        \
    } else if (mod_order == 8) {                                                            \
      QAM_ampa = set1_s16(QAM256_n1);                                                       \
      QAM_ampb = set1_s16(QAM256_n2);                                                       \
      QAM_ampc = set1_s16(QAM256_n3);                                                       \
    }                                                                                       \
                                                                                            \
    for (int aatx = 0; aatx < nb_layers; aatx++) {                                          \
      s16 *rxComp_s16 = (s16 *)&rxComp[aatx][symbol * buffer_length];                       \
      s16 *ch_maga_s16 = (s16 *)ch_maga[aatx];                                              \
      s16 *ch_magb_s16 = (s16 *)ch_magb[aatx];                                              \
      s16 *ch_magc_s16 = (s16 *)ch_magc[aatx];                                              \
      for (int aarx = 0; aarx < nb_rx_ant; aarx++) {                                        \
        s16 *rxF = (s16 *)rxFext[aarx];                                                     \
        s16 *chF = (s16 *)chFext[aatx][aarx];                                               \
                                                                                            \
        if (mod_order == 8) {                                                               \
          for (int i = 0; i < buffer_length >> COMPSHIFT; i++) {                            \
            s16 comp = oai_mm_cpx_mult_conj##SHIFT(chF[i], rxF[i]);                         \
            rxComp_s16[i] = adds_s16(rxComp_s16[i], comp);                                  \
            s16 mag = smadd_s16(chF[i], chF[i], SHIFT);                                     \
            mag = packs_s16(mag, mag);                                                      \
            mag = unpacklo_s16(mag, mag);                                                   \
            ch_maga_s16[i] = adds_s16(ch_maga_s16[i], mulhrs_s16(mag, QAM_ampa));           \
            ch_magb_s16[i] = adds_s16(ch_magb_s16[i], mulhrs_s16(mag, QAM_ampb));           \
            ch_magc_s16[i] = adds_s16(ch_magc_s16[i], mulhrs_s16(mag, QAM_ampc));           \
          }                                                                                 \
        } else if (mod_order == 6) {                                                        \
          for (int i = 0; i < buffer_length >> COMPSHIFT; i++) {                            \
            s16 comp = oai_mm_cpx_mult_conj##SHIFT(chF[i], rxF[i]);                         \
            rxComp_s16[i] = adds_s16(rxComp_s16[i], comp);                                  \
            s16 mag = smadd_s16(chF[i], chF[i], SHIFT);                                     \
            mag = packs_s16(mag, mag);                                                      \
            mag = unpacklo_s16(mag, mag);                                                   \
            ch_maga_s16[i] = adds_s16(ch_maga_s16[i], mulhrs_s16(mag, QAM_ampa));           \
            ch_magb_s16[i] = adds_s16(ch_magb_s16[i], mulhrs_s16(mag, QAM_ampb));           \
          }                                                                                 \
        } else if (mod_order == 4) {                                                        \
          for (int i = 0; i < buffer_length >> COMPSHIFT; i++) {                            \
            s16 comp = oai_mm_cpx_mult_conj##SHIFT(chF[i], rxF[i]);                         \
            rxComp_s16[i] = adds_s16(rxComp_s16[i], comp);                                  \
            s16 mag = smadd_s16(chF[i], chF[i], SHIFT);                                     \
            mag = packs_s16(mag, mag);                                                      \
            mag = unpacklo_s16(mag, mag);                                                   \
            ch_maga_s16[i] = adds_s16(ch_maga_s16[i], mulhrs_s16(mag, QAM_ampa));           \
          }                                                                                 \
        } else if (mod_order == 2) {                                                        \
          for (int i = 0; i < buffer_length >> COMPSHIFT; i++) {                            \
            s16 comp = oai_mm_cpx_mult_conj##SHIFT(chF[i], rxF[i]);                         \
            rxComp_s16[i] = adds_s16(rxComp_s16[i], comp);                                  \
          }                                                                                 \
        }                                                                                   \
        if (rho) {                                                                          \
          for (int atx = 0; atx < nb_layers; atx++) {                                       \
            s16 *rho_s16 = (s16 *)rho[aatx][atx];                                           \
            s16 *chF2 = (s16 *)chFext[atx][aarx];                                           \
            for (int i = 0; i < buffer_length >> COMPSHIFT; i++) {                          \
              rho_s16[i] = adds_s16(rho_s16[i], cpx_mult_conj_s16(chF[i], chF2[i], SHIFT)); \
            }                                                                               \
          }                                                                                 \
        }                                                                                   \
      }                                                                                     \
    }                                                                                       \
  }
COMP_SHIFT(1)
COMP_SHIFT(2)
COMP_SHIFT(3)
COMP_SHIFT(4)
COMP_SHIFT(5)
COMP_SHIFT(6)
COMP_SHIFT(7)
COMP_SHIFT(8)
COMP_SHIFT(9)
COMP_SHIFT(10)
COMP_SHIFT(11)
COMP_SHIFT(12)
COMP_SHIFT(13)
COMP_SHIFT(14)
COMP_SHIFT(15)

void nr_channel_compensation(uint32_t buffer_length,
                             int nb_rx_ant,
                             int nb_layers,
                             c16_t rxFext[nb_rx_ant][buffer_length],
                             c16_t chFext[nb_layers][nb_rx_ant][buffer_length],
                             c16_t ch_maga[nb_layers][buffer_length],
                             c16_t ch_magb[nb_layers][buffer_length],
                             c16_t ch_magc[nb_layers][buffer_length],
                             c16_t **rxComp,
                             c16_t (*rho)[nb_layers][buffer_length],
                             int mod_order,
                             uint32_t symbol,
                             uint32_t output_shift)
{
  switch (output_shift) {
    case 1:
      nr_channel_compensation1(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 2:
      nr_channel_compensation2(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 3:
      nr_channel_compensation3(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 4:
      nr_channel_compensation4(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 5:
      nr_channel_compensation5(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 6:
      nr_channel_compensation6(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 7:
      nr_channel_compensation7(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 8:
      nr_channel_compensation8(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 9:
      nr_channel_compensation9(buffer_length,
                               nb_rx_ant,
                               nb_layers,
                               rxFext,
                               chFext,
                               ch_maga,
                               ch_magb,
                               ch_magc,
                               rxComp,
                               rho,
                               mod_order,
                               symbol);
      break;
    case 10:
      nr_channel_compensation10(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    case 11:
      nr_channel_compensation11(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    case 12:
      nr_channel_compensation12(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    case 13:
      nr_channel_compensation13(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    case 14:
      nr_channel_compensation14(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    case 15:
      nr_channel_compensation15(buffer_length,
                                nb_rx_ant,
                                nb_layers,
                                rxFext,
                                chFext,
                                ch_maga,
                                ch_magb,
                                ch_magc,
                                rxComp,
                                rho,
                                mod_order,
                                symbol);
      break;
    default:
      LOG_E(NR_PHY, "Illegal shift %d\n", output_shift);
      break;
  }
}
#endif
