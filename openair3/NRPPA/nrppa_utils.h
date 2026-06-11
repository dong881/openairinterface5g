/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef NRPPA_UTILS_H_
#define NRPPA_UTILS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "common/5g_platform_types.h"
#include "common/platform_types.h"
#include "nrppa_messages_types.h"

void free_nrppa_trp_information_request(nrppa_trp_information_req_t *msg);
void free_nrppa_trp_information_response(nrppa_trp_information_resp_t *msg);
void free_nrppa_positioning_information_response(nrppa_positioning_information_resp_t *msg);
void free_nrppa_srs_carrier_list(nrppa_srs_carrier_list_t *srs_carrier_list);
void free_nrppa_positioning_activation_request(nrppa_positioning_activation_req_t *msg);
void free_nrppa_measurement_request(nrppa_measurement_req_t *msg);
void free_nrppa_measurement_resp(nrppa_measurement_resp_t *msg);

#endif /* NRPPA_UTILS_H_ */
