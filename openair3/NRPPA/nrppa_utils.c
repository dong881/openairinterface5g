/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "nrppa_utils.h"
#include "common/platform_types.h"

void free_nrppa_trp_information_response(nrppa_trp_information_resp_t *msg)
{
  nrppa_trp_information_list_t *trp_information_list = &msg->trp_information_list;
  uint32_t trp_info_item_length = trp_information_list->trp_information_item_length;
  for (int i = 0; i < trp_info_item_length; i++) {
    nrppa_trp_information_t *trp_information_item = &trp_information_list->trp_information_item[i];
    nrppa_trp_information_type_response_list_t *trp_info_type_resp_list = &trp_information_item->trp_information_type_response_list;
    free(trp_info_type_resp_list->trp_information_type_response_item);
  }
  free(msg->trp_information_list.trp_information_item);
}

void free_nrppa_trp_information_request(nrppa_trp_information_req_t *msg)
{
  if (msg->trp_information_type_list.trp_information_type_item) {
    free(msg->trp_information_type_list.trp_information_type_item);
  }
}

void free_nrppa_positioning_information_response(nrppa_positioning_information_resp_t *msg)
{
  /* SRS Configuration (O) */
  if (msg->srs_configuration) {
    nrppa_srs_carrier_list_t *srs_carrier_list = &msg->srs_configuration->srs_carrier_list;
    free_nrppa_srs_carrier_list(srs_carrier_list);
    free(msg->srs_configuration);
  }
}

void free_nrppa_srs_carrier_list(nrppa_srs_carrier_list_t *srs_carrier_list)
{
  uint32_t srs_carrier_list_len = srs_carrier_list->srs_carrier_list_length;
  for (int i = 0; i < srs_carrier_list_len; i++) {
    nrppa_srs_carrier_list_item_t *srs_carrier_list_item = &srs_carrier_list->srs_carrier_list_item[i];
    free(srs_carrier_list_item->uplink_channel_bw_per_scs_list.scs_specific_carrier);

    nrppa_active_ul_bwp_t *active_ul_bwp = &srs_carrier_list_item->active_ul_bwp;
    nrppa_srs_config_t *sRSConfig = &active_ul_bwp->srs_config;
    if (sRSConfig->srs_resource_list) {
      nrppa_srs_resource_list_t *srs_resource_list = sRSConfig->srs_resource_list;
      free(srs_resource_list->srs_resource);
      free(sRSConfig->srs_resource_list);
    }
    if (sRSConfig->pos_srs_resource_list) {
      nrppa_pos_srs_resource_list_t *pos_srs_resource_list = sRSConfig->pos_srs_resource_list;
      free(pos_srs_resource_list->pos_srs_resource_item);
      free(sRSConfig->pos_srs_resource_list);
    }
    if (sRSConfig->srs_resource_set_list) {
      nrppa_srs_resource_set_list_t *srs_resource_set_list = sRSConfig->srs_resource_set_list;
      uint32_t srs_resource_set_list_length = srs_resource_set_list->srs_resource_set_list_length;
      for (int j = 0; j < srs_resource_set_list_length; j++) {
        nrppa_srs_resource_set_t *srs_resource_set = &srs_resource_set_list->srs_resource_set[j];
        free(srs_resource_set->srs_resource_id_list.srs_resource_id);
      }
      free(srs_resource_set_list->srs_resource_set);
      free(sRSConfig->srs_resource_set_list);
    }
    if (sRSConfig->pos_srs_resource_set_list) {
      nrppa_pos_srs_resource_set_list_t *pos_srs_resource_set_list = sRSConfig->pos_srs_resource_set_list;
      uint32_t pos_srs_resource_set_list_length = pos_srs_resource_set_list->pos_srs_resource_set_list_length;
      for (int j = 0; j < pos_srs_resource_set_list_length; j++) {
        nrppa_pos_srs_resource_set_item_t *pos_srs_resource_set = &pos_srs_resource_set_list->pos_srs_resource_set_item[j];
        free(pos_srs_resource_set->pos_srs_resource_id_list.srs_pos_resource_id);
      }
      free(pos_srs_resource_set_list->pos_srs_resource_set_item);
      free(sRSConfig->pos_srs_resource_set_list);
    }
  }
  free(srs_carrier_list->srs_carrier_list_item);
}

void free_nrppa_positioning_activation_request(nrppa_positioning_activation_req_t *msg)
{
  if (msg->srs_type.present == NRPPA_SRS_TYPE_PR_SEMIPERSISTENTSRS) {
    free(msg->srs_type.choice.srs_resource_set_id);
  } else if (msg->srs_type.present == NRPPA_SRS_TYPE_PR_APERIODICSRS) {
    free(msg->srs_type.choice.aperiodic);
  }
}

void free_nrppa_measurement_request(nrppa_measurement_req_t *msg)
{
  free(msg->trp_measurement_request_list.trp_measurement_request_item);
  if (msg->measurement_quantities.measurement_quantities_item) {
    free(msg->measurement_quantities.measurement_quantities_item);
  }

  /* SRS Configuration (O) */
  if (msg->srs_configuration) {
    nrppa_srs_carrier_list_t *srs_carrier_list = &msg->srs_configuration->srs_carrier_list;
    free_nrppa_srs_carrier_list(srs_carrier_list);
    free(msg->srs_configuration);
  }
}

void free_nrppa_measurement_resp(nrppa_measurement_resp_t *msg)
{
  if (msg->measurement_response_list) {
    nrppa_measurement_response_list_t *list = msg->measurement_response_list;
    uint32_t meas_resp_list_len = list->measurement_response_list_length;
    for (int i = 0; i < meas_resp_list_len; i++) {
      nrppa_measurement_response_item_t *meas_response_item = &list->measurement_response_item[i];
      nrppa_measurement_result_t *MeasurementResult = &meas_response_item->measurement_result;
      uint32_t meas_result_length = MeasurementResult->measurement_result_item_length;
      for (int j = 0; j < meas_result_length; j++) {
        nrppa_measurement_result_item_t *item = &MeasurementResult->measurement_result_item[j];
        nrppa_measured_results_value_t *measuredResultsValue = &item->measured_results_value;
        if (measuredResultsValue->present == NRPPA_MEASURED_RESULTS_VALUE_PR_UL_ANGLEOFARRIVAL) {
          if (measuredResultsValue->choice.ul_angle_of_arrival.zenith_aoa) {
            free(measuredResultsValue->choice.ul_angle_of_arrival.zenith_aoa);
          }
        }
      }
      free(MeasurementResult->measurement_result_item);
    }
    free(msg->measurement_response_list->measurement_response_item);
    free(msg->measurement_response_list);
  }
}
