/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */
#include "nr_rrc_defs.h"
#include "openair2/COMMON/nrppa_messages_types.h"
#include "openair2/COMMON/f1ap_messages_types.h"

int rrc_gNB_process_trp_information_request(gNB_RRC_INST *rrc, const nrppa_trp_information_req_t *msg);
int rrc_CU_process_trp_information_response(f1ap_trp_information_resp_t *f1ap_msg);
