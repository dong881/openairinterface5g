/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */
#ifndef F1AP_CU_POSITIONING_H_
#define F1AP_CU_POSITIONING_H_

int CU_send_TRP_INFORMATION_REQUEST(sctp_assoc_t assoc_id, f1ap_trp_information_req_t *req);
int CU_handle_TRP_INFORMATION_RESPONSE(instance_t instance, sctp_assoc_t assoc_id, uint32_t stream, F1AP_F1AP_PDU_t *pdu);

#endif /* F1AP_CU_POSITIONING_H_ */
