/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */
#include "rrc_gNB_NRPPA.h"
#include "nr_rrc_defs.h"
#include "openair2/COMMON/nrppa_messages_types.h"
#include "openair2/COMMON/f1ap_messages_types.h"
#include "rrc_gNB_du.h"
#include "openair2/F1AP/lib/f1ap_positioning.h"

static nrppa_trp_reference_point_type_t f1ap2nrppa_reference_point_type(const f1ap_trp_reference_point_type_t *in)
{
  nrppa_trp_reference_point_type_t out = {0};
  switch (in->present) {
    case F1AP_TRP_REFERENCE_POINT_TYPE_PR_TRPPOSITION_RELATIVE_GEODETIC:
      out.present = NRPPA_TRP_REFERENCE_POINT_TYPE_PR_TRPPOSITION_RELATIVE_GEODETIC;
      const f1ap_relative_geodetic_location_t *f1_tRPPositionRelativeGeodetic = &in->choice.trp_position_relative_geodetic;
      nrppa_relative_geodetic_location_t *tRPPositionRelativeGeodetic = &out.choice.trp_position_relative_geodetic;
      tRPPositionRelativeGeodetic->milli_arc_second_units = f1_tRPPositionRelativeGeodetic->milli_arc_second_units;
      tRPPositionRelativeGeodetic->height_units = f1_tRPPositionRelativeGeodetic->height_units;
      tRPPositionRelativeGeodetic->delta_latitude = f1_tRPPositionRelativeGeodetic->delta_latitude;
      tRPPositionRelativeGeodetic->delta_longitude = f1_tRPPositionRelativeGeodetic->delta_longitude;
      tRPPositionRelativeGeodetic->delta_height = f1_tRPPositionRelativeGeodetic->delta_height;

      const f1ap_location_uncertainty_t *f1_locationUncertainty_g = &f1_tRPPositionRelativeGeodetic->location_uncertainty;
      nrppa_location_uncertainty_t *locationUncertainty_g = &tRPPositionRelativeGeodetic->location_uncertainty;
      locationUncertainty_g->horizontal_uncertainty = f1_locationUncertainty_g->horizontal_uncertainty;
      locationUncertainty_g->horizontal_confidence = f1_locationUncertainty_g->horizontal_confidence;
      locationUncertainty_g->vertical_uncertainty = f1_locationUncertainty_g->vertical_uncertainty;
      locationUncertainty_g->vertical_confidence = f1_locationUncertainty_g->vertical_confidence;
      break;
    case F1AP_TRP_REFERENCE_POINT_TYPE_PR_TRPPOSITION_RELATIVE_CARTESIAN:
      out.present = NRPPA_TRP_REFERENCE_POINT_TYPE_PR_TRPPOSITION_RELATIVE_CARTESIAN;
      const f1ap_relative_cartesian_location_t *f1_tRPPositionRelativeCartesian = &in->choice.trp_position_relative_cartesian;
      nrppa_relative_cartesian_location_t *tRPPositionRelativeCartesian = &out.choice.trp_position_relative_cartesian;
      tRPPositionRelativeCartesian->xyz_unit = f1_tRPPositionRelativeCartesian->xyz_unit;
      tRPPositionRelativeCartesian->xvalue = f1_tRPPositionRelativeCartesian->xvalue;
      tRPPositionRelativeCartesian->yvalue = f1_tRPPositionRelativeCartesian->yvalue;
      tRPPositionRelativeCartesian->zvalue = f1_tRPPositionRelativeCartesian->zvalue;

      const f1ap_location_uncertainty_t *f1_locationUncertainty_c = &f1_tRPPositionRelativeCartesian->location_uncertainty;
      nrppa_location_uncertainty_t *locationUncertainty_c = &tRPPositionRelativeCartesian->location_uncertainty;
      locationUncertainty_c->horizontal_uncertainty = f1_locationUncertainty_c->horizontal_uncertainty;
      locationUncertainty_c->horizontal_confidence = f1_locationUncertainty_c->horizontal_confidence;
      locationUncertainty_c->vertical_uncertainty = f1_locationUncertainty_c->vertical_uncertainty;
      locationUncertainty_c->vertical_confidence = f1_locationUncertainty_c->vertical_confidence;
      break;
    default:
      AssertFatal(false, "illegal trp reference point type entry %d\n", in->present);
      break;
  }
  return out;
}

static nrppa_ngran_high_accuracy_access_point_position_t f1ap2nrppa_trp_ha_pos(
    const f1ap_ngran_high_accuracy_access_point_position_t *in)
{
  nrppa_ngran_high_accuracy_access_point_position_t out = {0};
  out.latitude = in->latitude;
  out.longitude = in->longitude;
  out.altitude = in->altitude;
  out.uncertainty_semi_major = in->uncertainty_semi_major;
  out.uncertainty_semi_minor = in->uncertainty_semi_minor;
  out.orientation_of_major_axis = in->orientation_of_major_axis;
  out.horizontal_confidence = in->horizontal_confidence;
  out.uncertainty_altitude = in->uncertainty_altitude;
  out.vertical_confidence = in->vertical_confidence;
  return out;
}

static nrppa_geographical_coordinates_t f1ap2nrppa_geographical_coordinates(const f1ap_geographical_coordinates_t *in)
{
  nrppa_geographical_coordinates_t out = {0};
  const f1ap_trp_position_definition_type_t *f1_trp_pos_def_type = &in->trp_position_definition_type;
  nrppa_trp_position_definition_type_t *trp_pos_def_type = &out.trp_position_definition_type;
  switch (f1_trp_pos_def_type->present) {
    case F1AP_TRP_POSITION_DEFINITION_TYPE_PR_NOTHING:
      trp_pos_def_type->present = NRPPA_TRP_POSITION_DEFINITION_TYPE_PR_NOTHING;
      break;
    case F1AP_TRP_POSITION_DEFINITION_TYPE_PR_DIRECT:
      trp_pos_def_type->present = NRPPA_TRP_POSITION_DEFINITION_TYPE_PR_DIRECT;
      const f1ap_trp_position_direct_t *f1_direct = &f1_trp_pos_def_type->choice.direct;
      nrppa_trp_position_direct_t *direct = &trp_pos_def_type->choice.direct;

      if (f1_direct->accuracy.present == F1AP_TRP_POSITION_DIRECT_ACCURACY_PR_TRPPOSITION) {
        direct->accuracy.present = NRPPA_TRP_POSITION_DIRECT_ACCURACY_PR_TRPPOSITION;
        nrppa_access_point_position_t *trp_pos = &direct->accuracy.choice.trp_position;
        const f1ap_access_point_position_t *f1_trp_pos = &f1_direct->accuracy.choice.trp_position;
        trp_pos->latitude_sign = f1_trp_pos->latitude_sign;
        trp_pos->latitude = f1_trp_pos->latitude;
        trp_pos->longitude = f1_trp_pos->longitude;
        trp_pos->direction_of_altitude = f1_trp_pos->direction_of_altitude;
        trp_pos->altitude = f1_trp_pos->altitude;
        trp_pos->uncertainty_semi_major = f1_trp_pos->uncertainty_semi_major;
        trp_pos->uncertainty_semi_minor = f1_trp_pos->uncertainty_semi_minor;
        trp_pos->orientation_of_major_axis = f1_trp_pos->orientation_of_major_axis;
        trp_pos->uncertainty_altitude = f1_trp_pos->uncertainty_altitude;
        trp_pos->confidence = f1_trp_pos->confidence;
      } else if (f1_direct->accuracy.present == F1AP_TRP_POSITION_DIRECT_ACCURACY_PR_TRPHAPOSITION) {
        direct->accuracy.present = NRPPA_TRP_POSITION_DIRECT_ACCURACY_PR_TRPHAPOSITION;
        nrppa_ngran_high_accuracy_access_point_position_t *trp_ha_pos = &direct->accuracy.choice.trp_HAposition;
        const f1ap_ngran_high_accuracy_access_point_position_t *f1_trp_ha_pos = &f1_direct->accuracy.choice.trp_HAposition;
        *trp_ha_pos = f1ap2nrppa_trp_ha_pos(f1_trp_ha_pos);
      } else {
        AssertFatal(false, "illegal direct accuracy entry %d\n", direct->accuracy.present);
      }
      break;
    case F1AP_TRP_POSITION_DEFINITION_TYPE_PR_REFERENCED:
      trp_pos_def_type->present = NRPPA_TRP_POSITION_DEFINITION_TYPE_PR_REFERENCED;
      const f1ap_trp_position_referenced_t *f1_referenced = &f1_trp_pos_def_type->choice.referenced;
      nrppa_trp_position_referenced_t *referenced = &trp_pos_def_type->choice.referenced;
      const f1ap_reference_point_t *f1_referencePoint = &f1_referenced->reference_point;
      nrppa_reference_point_t *referencePoint = &referenced->reference_point;

      if (f1_referencePoint->present == F1AP_REFERENCE_POINT_PR_COORDINATEID) {
        referencePoint->present = NRPPA_REFERENCE_POINT_PR_COORDINATEID;
        referencePoint->choice.coordinate_id = f1_referencePoint->choice.coordinate_id;
      } else if (f1_referencePoint->present == F1AP_REFERENCE_POINT_PR_REFERENCEPOINTCOORDINATE) {
        referencePoint->present = NRPPA_REFERENCE_POINT_PR_REFERENCEPOINTCOORDINATE;
        nrppa_access_point_position_t *referencePointCoordinate = &referencePoint->choice.reference_point_coordinate;
        const f1ap_access_point_position_t *f1_referencePointCoordinate = &f1_referencePoint->choice.reference_point_coordinate;
        referencePointCoordinate->latitude_sign = f1_referencePointCoordinate->latitude_sign;
        referencePointCoordinate->latitude = f1_referencePointCoordinate->latitude;
        referencePointCoordinate->longitude = f1_referencePointCoordinate->longitude;
        referencePointCoordinate->direction_of_altitude = f1_referencePointCoordinate->direction_of_altitude;
        referencePointCoordinate->altitude = f1_referencePointCoordinate->altitude;
        referencePointCoordinate->uncertainty_semi_major = f1_referencePointCoordinate->uncertainty_semi_major;
        referencePointCoordinate->uncertainty_semi_minor = f1_referencePointCoordinate->uncertainty_semi_minor;
        referencePointCoordinate->orientation_of_major_axis = f1_referencePointCoordinate->orientation_of_major_axis;
        referencePointCoordinate->uncertainty_altitude = f1_referencePointCoordinate->uncertainty_altitude;
        referencePointCoordinate->confidence = f1_referencePointCoordinate->confidence;
      } else if (f1_referencePoint->present == F1AP_REFERENCE_POINT_PR_REFERENCEPOINTCOORDINATEHA) {
        referencePoint->present = NRPPA_REFERENCE_POINT_PR_REFERENCEPOINTCOORDINATEHA;
        nrppa_ngran_high_accuracy_access_point_position_t *referencePointCoordinateHA =
            &referencePoint->choice.reference_point_coordinateHA;
        const f1ap_ngran_high_accuracy_access_point_position_t *f1_referencePointCoordinateHA =
            &f1_referencePoint->choice.reference_point_coordinateHA;
        *referencePointCoordinateHA = f1ap2nrppa_trp_ha_pos(f1_referencePointCoordinateHA);
      } else {
        AssertFatal(false, "illegal reference point entry %d\n", referencePoint->present);
      }

      const f1ap_trp_reference_point_type_t *f1_referencePointType = &f1_referenced->reference_point_type;
      nrppa_trp_reference_point_type_t *referencePointType = &referenced->reference_point_type;
      *referencePointType = f1ap2nrppa_reference_point_type(f1_referencePointType);
      break;
    default:
      AssertFatal(false, "illegal Geographical Coordinates entry\n");
      break;
  }
  return out;
}

static nrppa_trp_information_type_response_item_t f1ap2nrppa_trp_info_type_response_item(
    const f1ap_trp_information_type_response_item_t *in)
{
  nrppa_trp_information_type_response_item_t out = {0};
  switch (in->present) {
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NOTHING:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NOTHING;
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_PCI_NR:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_PCI_NR;
      out.choice.pci_nr = in->choice.pci_nr;
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NG_RAN_CGI:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NG_RAN_CGI;
      out.choice.ng_ran_cgi.plmn.mcc = in->choice.ng_ran_cgi.plmn.mcc;
      out.choice.ng_ran_cgi.plmn.mnc = in->choice.ng_ran_cgi.plmn.mnc;
      out.choice.ng_ran_cgi.plmn.mnc_digit_length = in->choice.ng_ran_cgi.plmn.mnc_digit_length;
      out.choice.ng_ran_cgi.nr_cellid = in->choice.ng_ran_cgi.nr_cellid;
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NRARFCN:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_NRARFCN;
      out.choice.nr_arfcn = in->choice.nr_arfcn;
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_PRSCONFIGURATION:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_PRSCONFIGURATION;
      AssertFatal(false, "TRP information type response item PRS configuration unsupported\n");
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SSBINFORMATION:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SSBINFORMATION;
      AssertFatal(false, "TRP information type response item SSB Information unsupported\n");
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SFNINITIALISATIONTIME:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SFNINITIALISATIONTIME;
      AssertFatal(false, "TRP information type response item SFN Initialization Time unsupported\n");
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SPATIALDIRECTIONINFORMATION:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_SPATIALDIRECTIONINFORMATION;
      AssertFatal(false, "TRP information type response item Spatial Direction Information unsupported\n");
      break;
    case F1AP_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_GEOGRAPHICALCOORDINATES:
      out.present = NRPPA_TRP_INFORMATION_TYPE_RESPONSE_ITEM_PR_GEOGRAPHICALCOORDINATES;
      out.choice.geographical_coordinates = f1ap2nrppa_geographical_coordinates(&in->choice.geographical_coordinates);
      break;
    default:
      AssertFatal(false, "received illegal trp information type response item %d\n", in->present);
      break;
  }
  return out;
}

int rrc_gNB_process_trp_information_request(gNB_RRC_INST *rrc, const nrppa_trp_information_req_t *msg)
{
  f1ap_trp_information_req_t f1ap_msg = {0};
  f1ap_msg.transaction_id = msg->transaction_id;

  if (msg->has_trp_list) {
    f1ap_msg.has_trp_list = true;
    uint32_t list_len = msg->trp_list.trp_list_length;
    f1ap_msg.trp_list.trp_list_length = list_len;
    f1ap_msg.trp_list.trp_list_item = calloc_or_fail(list_len, sizeof(*f1ap_msg.trp_list.trp_list_item));
    f1ap_trp_list_item_t *f1ap_item = f1ap_msg.trp_list.trp_list_item;
    const nrppa_trp_list_item_t *nrppa_item = msg->trp_list.trp_list_item;
    for (int i = 0; i < list_len; i++) {
      f1ap_item[i].trp_id = nrppa_item[i].trp_id;
    }
  }

  const nrppa_trp_information_type_list_t *nrppa_info_type_list = &msg->trp_information_type_list;
  f1ap_trp_information_type_list_t *f1ap_info_type_list = &f1ap_msg.trp_information_type_list;
  uint8_t info_type_list_len = nrppa_info_type_list->trp_information_type_list_length;
  DevAssert(info_type_list_len >= 0);
  if (info_type_list_len > 0) {
    f1ap_info_type_list->trp_information_type_list_length = info_type_list_len;
    f1ap_info_type_list->trp_information_type_item =
        calloc_or_fail(info_type_list_len, sizeof(*f1ap_info_type_list->trp_information_type_item));
    nrppa_trp_information_type_item_pr *nrppa_info_item = nrppa_info_type_list->trp_information_type_item;
    f1ap_trp_information_type_item_pr *f1ap_info_item = f1ap_info_type_list->trp_information_type_item;
    for (int i = 0; i < info_type_list_len; i++) {
      switch (nrppa_info_item[i]) {
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_NR_PCI:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_NR_PCI;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_NG_RAN_CGI:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_NG_RAN_CGI;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_NR_ARFCN:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_NR_ARFCN;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_PRS_CONFIG:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_PRS_CONFIG;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_SSB_CONFIG:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_SSB_CONFIG;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_SFN_INIT_TIME:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_SFN_INIT_TIME;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_SPATIAL_DIRECTION_INFO:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_SPATIAL_DIRECTION_INFO;
          break;
        case NRPPA_TRP_INFORMATION_TYPE_ITEM_GEO_COORDINATES:
          f1ap_info_item[i] = F1AP_TRP_INFORMATION_TYPE_ITEM_GEO_COORDINATES;
          break;
        default:
          AssertFatal(false, "Illegal TRP Information Type\n");
          break;
      }
    }
  } else {
    // HACK: Made to work with OAI
    // We fill NG_RAN_CGI and GEO_COORDINATES as default
    info_type_list_len = 2;
    f1ap_info_type_list->trp_information_type_list_length = info_type_list_len;
    f1ap_info_type_list->trp_information_type_item =
        calloc_or_fail(info_type_list_len, sizeof(*f1ap_info_type_list->trp_information_type_item));
    f1ap_trp_information_type_item_pr *f1ap_info_item = f1ap_info_type_list->trp_information_type_item;
    f1ap_info_item[0] = F1AP_TRP_INFORMATION_TYPE_ITEM_NG_RAN_CGI;
    f1ap_info_item[1] = F1AP_TRP_INFORMATION_TYPE_ITEM_GEO_COORDINATES;
  }

  rrc_send_trp_information_request_to_dus(rrc, &f1ap_msg);
  free_trp_information_req(&f1ap_msg);
  return 0;
}

int rrc_CU_process_trp_information_response(f1ap_trp_information_resp_t *f1ap_msg)
{
  MessageDef *msg_resp = itti_alloc_new_message(TASK_RRC_GNB, 0, NRPPA_TRP_INFORMATION_RESP);
  nrppa_trp_information_resp_t *nrppa_msg = &NRPPA_TRP_INFORMATION_RESP(msg_resp);
  nrppa_msg->transaction_id = f1ap_msg->transaction_id;

  uint32_t trp_info_item_length = f1ap_msg->trp_information_list.trp_information_item_length;
  AssertFatal(trp_info_item_length > 0, "at least 1 TRP Information Item must be present\n");
  nrppa_msg->trp_information_list.trp_information_item_length = trp_info_item_length;
  nrppa_msg->trp_information_list.trp_information_item =
      calloc_or_fail(trp_info_item_length, sizeof(*nrppa_msg->trp_information_list.trp_information_item));

  nrppa_trp_information_t *trp_information_item_nrppa = nrppa_msg->trp_information_list.trp_information_item;
  f1ap_trp_information_t *trp_information_item_f1ap = f1ap_msg->trp_information_list.trp_information_item;

  for (int i = 0; i < trp_info_item_length; i++) {
    uint8_t trp_info_type_resp_item_len =
        trp_information_item_f1ap[i].trp_information_type_response_list.trp_information_type_response_item_length;
    AssertFatal(trp_info_type_resp_item_len > 0, "at least 1 TRP Information type response Item must be present\n");

    trp_information_item_nrppa[i].trp_id = trp_information_item_f1ap[i].trp_id;

    nrppa_trp_information_type_response_list_t *trp_info_type_resp_list_nrppa =
        &trp_information_item_nrppa[i].trp_information_type_response_list;
    f1ap_trp_information_type_response_list_t *trp_info_type_resp_list_f1ap =
        &trp_information_item_f1ap[i].trp_information_type_response_list;
    trp_info_type_resp_list_nrppa->trp_information_type_response_item_length = trp_info_type_resp_item_len;
    trp_info_type_resp_list_nrppa->trp_information_type_response_item =
        calloc_or_fail(trp_info_type_resp_item_len, sizeof(*trp_info_type_resp_list_nrppa->trp_information_type_response_item));
    for (int j = 0; j < trp_info_type_resp_item_len; j++) {
      trp_info_type_resp_list_nrppa->trp_information_type_response_item[j] =
          f1ap2nrppa_trp_info_type_response_item(&trp_info_type_resp_list_f1ap->trp_information_type_response_item[j]);
    }
  }

  LOG_I(NR_RRC, "Sending NRPPA_TRP_INFORMATION_RESP to TASK_NRPPA\n");
  itti_send_msg_to_task(TASK_NRPPA, 0, msg_resp);

  return 0;
}
