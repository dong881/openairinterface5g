/* SPDX-License-Identifier: LicenseRef-CSSL-1.0 */

#include "nfapi/oai_integration/nfapi_pnf_pacing.h"

#include <assert.h>
#include <stdio.h>

static void assert_timespec(struct timespec actual, time_t sec, long nsec)
{
  assert(actual.tv_sec == sec);
  assert(actual.tv_nsec == nsec);
}

static void test_slot_durations(void)
{
  static const long expected[] = {1000000L, 500000L, 250000L, 125000L, 62500L};
  for (unsigned int mu = 0; mu < sizeof(expected) / sizeof(expected[0]); ++mu)
    assert(nfapi_pnf_slot_duration_ns(mu) == expected[mu]);
  assert(nfapi_pnf_slot_duration_ns(5) == 0);
}

static void test_regular_deadlines(void)
{
  nfapi_pnf_pacer_t pacer = {0};
  struct timespec deadline = {0};

  assert(!nfapi_pnf_pacer_next(&pacer, 1, (struct timespec){.tv_sec = 10, .tv_nsec = 999800000L}, &deadline));
  assert(nfapi_pnf_pacer_next(&pacer, 1, (struct timespec){.tv_sec = 10, .tv_nsec = 999900000L}, &deadline));
  assert_timespec(deadline, 11, 300000L);

  for (int slot = 2; slot <= 2000; ++slot) {
    const struct timespec now = deadline;
    assert(nfapi_pnf_pacer_next(&pacer, 1, now, &deadline));
  }
  assert_timespec(deadline, 11, 999800000L);
}

static void test_missed_deadline_rebases(void)
{
  nfapi_pnf_pacer_t pacer = {0};
  struct timespec deadline = {0};

  assert(!nfapi_pnf_pacer_next(&pacer, 0, (struct timespec){0}, &deadline));
  assert(!nfapi_pnf_pacer_next(&pacer, 0, (struct timespec){.tv_nsec = 1500000L}, &deadline));
  assert_timespec(pacer.next_slot, 0, 1500000L);

  assert(nfapi_pnf_pacer_next(&pacer, 0, (struct timespec){.tv_nsec = 1600000L}, &deadline));
  assert_timespec(deadline, 0, 2500000L);
}

int main(void)
{
  test_slot_durations();
  test_regular_deadlines();
  test_missed_deadline_rebases();
  puts("RFsim PNF pacing tests passed");
  return 0;
}
