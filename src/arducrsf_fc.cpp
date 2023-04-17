#include <Arduino.h>
#include "arducrsf_fc.h"
// =======
// PUBLIC
// =======
CRSF_FC::CRSF_FC()
{
    port = NULL;
}

CRSF_FC::CRSF_FC(Stream *port)
{
    this->port = port;
}

bool CRSF_FC::set_port(Stream *port)
{
    this->port = port;
    return true;
}

bool CRSF_FC::read_pkt()
{
    if (port && port->available())
    {
        // dest
        bool dest_found = false;
        for (uint8_t i = 0; i < CRSF_LEN_MAX; i++)
        {
            *rx.dest = port->read();
            if (dest_check())
            {
                dest_found = true;
                break;
            }
        }
        if (dest_found)
        {
            // len
            *rx.len = port->read();
            if (len_check())
            {
                // type
                *rx.type = port->read();
                if (type_check())
                {
                    // payload
                    port->readBytes(rx.payload, *rx.len - 2);
                    // crc8
                    rx.crc8 = rx.len + *rx.len;
                    *rx.crc8 = port->read();
                    if (crc8_check())
                    {
                        return true;
                    }
                }
            }
        }
    }
    *rx.type = 0x00; // This allows to "reset" a packet received earlier
    return false;
}

uint8_t CRSF_FC::get_pkt_type()
{
    if (*rx.type)
    {
        return *rx.type;
    }
    return false;
}

bool CRSF_FC::get_default_packet(crsf_default *pkt)
{
    if (*rx.type)
    {
        memcpy(pkt->packet, rx.packet, CRSF_LEN_MAX);
        pkt->crc8 = pkt->len + *pkt->len;
    }
    return false;
}

bool CRSF_FC::get_rc_channels(crsf_rc_channels *pkt, bool to_fc_range)
{
    if (*rx.type == CRSF_FRAMETYPE_RC_CHANNELS)
    {
        pkt->dest = *rx.dest;
        pkt->len = *rx.len;
        pkt->type = *rx.type;

        pkt->ch[0] = ((rx.payload[0] | rx.payload[1] << 8) & 0x07FF);
        pkt->ch[1] = ((rx.payload[1] >> 3 | rx.payload[2] << 5) & 0x07FF);
        pkt->ch[2] = ((rx.payload[2] >> 6 | rx.payload[3] << 2 | rx.payload[4] << 10) & 0x07FF);
        pkt->ch[3] = ((rx.payload[4] >> 1 | rx.payload[5] << 7) & 0x07FF);
        pkt->ch[4] = ((rx.payload[5] >> 4 | rx.payload[6] << 4) & 0x07FF);
        pkt->ch[5] = ((rx.payload[6] >> 7 | rx.payload[7] << 1 | rx.payload[8] << 9) & 0x07FF);
        pkt->ch[6] = ((rx.payload[8] >> 2 | rx.payload[9] << 6) & 0x07FF);
        pkt->ch[7] = ((rx.payload[9] >> 5 | rx.payload[10] << 3) & 0x07FF);
        pkt->ch[8] = ((rx.payload[11] | rx.payload[12] << 8) & 0x07FF);
        pkt->ch[9] = ((rx.payload[12] >> 3 | rx.payload[13] << 5) & 0x07FF);
        pkt->ch[10] = ((rx.payload[13] >> 6 | rx.payload[14] << 2 | rx.payload[15] << 10) & 0x07FF);
        pkt->ch[11] = ((rx.payload[15] >> 1 | rx.payload[16] << 7) & 0x07FF);
        pkt->ch[12] = ((rx.payload[16] >> 4 | rx.payload[17] << 4) & 0x07FF);
        pkt->ch[13] = ((rx.payload[17] >> 7 | rx.payload[18] << 1 | rx.payload[19] << 9) & 0x07FF);
        pkt->ch[14] = ((rx.payload[19] >> 2 | rx.payload[20] << 6) & 0x07FF);
        pkt->ch[15] = ((rx.payload[20] >> 5 | rx.payload[21] << 3) & 0x07FF);

        pkt->crc8 = *rx.crc8;
        if (to_fc_range)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                pkt->ch[i] = floor(0.610873549 * pkt->ch[i] + 894.208002443);
            }
            for (uint8_t i = 4; i < 16; i++)
            {
                pkt->ch[i] = floor(0.624609619 * pkt->ch[i] + 881.199562773);
            }
        }
        return true;
    }
    return false;
}

// ========
// PRIVATE
// ========
bool CRSF_FC::dest_check()
{
    if (*rx.dest == CRSF_ADDRESS_FLIGHT_CONTROLLER ||
        *rx.dest == CRSF_ADDRESS_CRSF_RECEIVER)
    {
        return true;
    }
    return false;
}

bool CRSF_FC::len_check()
{
    if (*rx.len > CRSF_LEN_MAX)
    {
        return false;
    }
    return true;
}

bool CRSF_FC::type_check()
{
    if (*rx.type == CRSF_FRAMETYPE_RC_CHANNELS ||
        *rx.type == CRSF_FRAMETYPE_DEVICE_PING ||
        *rx.type == CRSF_FRAMETYPE_DEVICE_INFO ||
        *rx.type == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY ||
        *rx.type == CRSF_FRAMETYPE_PARAMETER_READ ||
        *rx.type == CRSF_FRAMETYPE_PARAMETER_WRITE ||
        *rx.type == CRSF_FRAMETYPE_COMMAND)
    {
        return true;
    }
    return false;
}

bool CRSF_FC::crc8_check()
{
    uint8_t crc8_calculated = crc8_calc();
    return *rx.crc8 == crc8_calculated;
}

uint8_t CRSF_FC::crc8_calc()
{
    static const uint8_t crc8_table[256] = {
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};
    uint8_t *type_payload = &rx.packet[2];
    uint8_t result = 0;
    for (uint8_t i = 0; i < (*rx.len - 1); i++)
    {
        result = crc8_table[result ^ type_payload[i]];
    }
    return result;
}