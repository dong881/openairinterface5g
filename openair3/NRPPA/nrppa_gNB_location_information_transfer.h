/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef NRPPA_GNB_POSITIONING_PROCEDURES_H_
#define NRPPA_GNB_POSITIONING_PROCEDURES_H_

int nrppa_gNB_handle_trp_information_request(nrppa_gnb_ue_info_t *nrppa_msg_info, const NRPPA_NRPPA_PDU_t *pdu);
int nrppa_gNB_trp_information_response(instance_t instance, MessageDef *msg_p);
NRPPA_TRPInformationItem_t encode_trp_info_type_response_item_nrppa(nrppa_trp_information_type_response_item_t *in);
int nrppa_gNB_handle_positioning_information_request(nrppa_gnb_ue_info_t *nrppa_msg_info, const NRPPA_NRPPA_PDU_t *pdu);
int nrppa_gNB_positioning_information_response(instance_t instance, MessageDef *msg_p);
NRPPA_SRSCarrier_List_t encode_srs_carrier_list_nrppa(const nrppa_srs_carrier_list_t *in_list);
int nrppa_gNB_handle_positioning_activation_request(nrppa_gnb_ue_info_t *nrppa_msg_info, const NRPPA_NRPPA_PDU_t *pdu);
void decode_nrppa_srstype(NRPPA_SRSType_t *srs_type, nrppa_srs_type_t *out);
int nrppa_gNB_positioning_activation_response(instance_t instance, MessageDef *msg_p);

#endif /* NRPPA_GNB_POSITIONING_PROCEDURES_H_ */
