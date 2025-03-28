/*
  generic transport class for handling OpenDroneID messages
 */
#include <Arduino.h>
#include "transport.h"
#include "parameters.h"
#include "util.h"
#include "monocypher.h"

const char *Transport::parse_fail = "uninitialised";
struct Transport::ack Transport::ack_response = {0,0,0};

uint32_t Transport::last_location_ms;
uint32_t Transport::last_basic_id_ms;
uint32_t Transport::last_self_id_ms;
uint32_t Transport::last_operator_id_ms;
uint32_t Transport::last_system_ms;
uint32_t Transport::last_flt_time_ms;
uint32_t Transport::last_serial_number_ms;
uint32_t Transport::last_system_timestamp;
float Transport::last_location_timestamp;

mavlink_open_drone_id_location_t Transport::location;
mavlink_open_drone_id_basic_id_t Transport::basic_id;
mavlink_open_drone_id_authentication_t Transport::authentication;
mavlink_open_drone_id_self_id_t Transport::self_id;
mavlink_open_drone_id_system_t Transport::system;
mavlink_open_drone_id_operator_id_t Transport::operator_id;
mavlink_aurelia_flt_time_t Transport::flt_time;
mavlink_aurelia_odid_serial_number_t Transport::serial_number;
mavlink_aurelia_util_ack_request_t Transport::ack_request;
uint8_t Transport::fl_status = 0;

Transport::Transport()
{
}

/*
  check we are OK to fly
 */
uint8_t Transport::status_check(const char *&reason)
{
    uint8_t status = MAV_AURELIA_CHECK_STATUS_FAIL_FLYING_NOT_ALLOWED;

     //return status OK if we have enabled the force arm option
    if ((g.options & OPTIONS_FORCE_ARM_OK)) {
        if(reason == nullptr){
            status = MAV_AURELIA_CHECK_STATUS_GOOD_TO_ARM;
        }
        fl_status = status;
        return status;
    }

    const uint32_t max_age_location_ms = 3000;
    const uint32_t max_age_other_ms = 22000;
    const uint32_t now_ms = millis();

    String ret = "";

    if (last_location_ms == 0 || now_ms - last_location_ms > max_age_location_ms || location.latitude == 0 && location.longitude == 0) {
        ret += "LOC ";
    }
    if (!g.have_basic_id_info()) {
        // if there is no basic ID data stored in the parameters give warning. If basic ID data are streamed to RID device,
        // it will store them in the parameters
        ret += "ID ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }

    if ((last_self_id_ms == 0  || now_ms - last_self_id_ms > max_age_other_ms)) {
        ret += "SELF_ID ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }

    if ((last_operator_id_ms == 0 || now_ms - last_operator_id_ms > max_age_other_ms)) {
        ret += "OP_ID ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }

    if ((last_system_ms == 0 || now_ms - last_system_ms > max_age_location_ms)) {
        // we use location age limit for system as the operator location needs to come in as fast
        // as the vehicle location for FAA standard
        ret += "SYS ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }

    if ((system.operator_latitude == 0 && system.operator_longitude == 0)) {
        ret += "OP_LOC ";
    }
    if ((serial_number.serial_number == 0 || now_ms - last_serial_number_ms > max_age_other_ms)) {
        ret += "SN ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }else if (serial_number.serial_number != g.get_serial_number()) {
        ret += "BAD_RID ";
        status = MAV_AURELIA_CHECK_STATUS_FAIL_GENERIC;
    }

    if (ret.length() == 0 && reason == nullptr) {
        status = MAV_AURELIA_CHECK_STATUS_GOOD_TO_ARM;
    } else {
        static char return_string[200];
        memset(return_string, 0, sizeof(return_string));
        if (reason != nullptr) {
            strlcpy(return_string, reason, sizeof(return_string));
        }
        strlcat(return_string, ret.c_str(), sizeof(return_string));
        reason = return_string;
    }

    fl_status = status;

    return status;
}

void Transport::set_ack_response(uint8_t mac[MAC_LENGTH], uint8_t nonce[NONCE_LENGTH], uint8_t cipher_text[MSG_LENGTH]){
    memcpy(ack_response.mac, mac, sizeof(ack_response.mac));
    memcpy(ack_response.nonce, nonce, sizeof(ack_response.nonce));
    memcpy(ack_response.cipher_text, cipher_text, sizeof(ack_response.cipher_text));
}

/*
  make a session key
 */
void Transport::make_session_key(uint8_t key[8]) const
{
    struct {
        uint32_t time_us;
        uint8_t mac[8];
        uint32_t rand;
    } data {};
    static_assert(sizeof(data) % 4 == 0, "data must be multiple of 4 bytes");

    esp_efuse_mac_get_default(data.mac);
    data.time_us = micros();
    data.rand = random(0xFFFFFFFF);
    const uint64_t c64 = crc_crc64((const uint32_t *)&data, sizeof(data)/sizeof(uint32_t));
    memcpy(key, (uint8_t *)&c64, 8);
}

/*
  check signature in a command against public keys
 */
bool Transport::check_signature(uint8_t sig_length, uint8_t data_len, uint32_t sequence, uint32_t operation,
                                const uint8_t *data)
{
    if (g.no_public_keys()) {
        // allow through if no keys are setup
        return true;
    }
    if (sig_length != 64) {
        // monocypher signatures are 64 bytes
        return false;
    }

    /*
      loop over all public keys, if one matches then we are OK
     */
    for (uint8_t i=0; i<MAX_PUBLIC_KEYS; i++) {
        uint8_t key[32];
        if (!g.get_public_key(i, key)) {
            continue;
        }
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        crypto_check_init(actx, &data[data_len], key);

        crypto_check_update(actx, (const uint8_t*)&sequence, sizeof(sequence));
        crypto_check_update(actx, (const uint8_t*)&operation, sizeof(operation));
        crypto_check_update(actx, data, data_len);
        if (operation != SECURE_COMMAND_GET_SESSION_KEY &&
            operation != SECURE_COMMAND_GET_REMOTEID_SESSION_KEY) {
            crypto_check_update(actx, session_key, sizeof(session_key));
        }
        if (crypto_check_final(actx) == 0) {
            // good signature
            return true;
        }
    }
    return false;
}
