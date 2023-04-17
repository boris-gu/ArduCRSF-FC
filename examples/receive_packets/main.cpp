#include <Arduino.h>
#include "arducrsf_fc.h"

CRSF_FC ep2;
crsf_default def_pkt;
crsf_rc_channels channels;

void setup()
{
    // USB
    Serial.begin(115200);
    while (!Serial)
    {
    }
    // ELRS
    Serial2.begin(115200);
    while (!Serial2)
    {
        Serial.println("EP2 waiting");
        delay(500);
    }
    ep2.set_port(&Serial2);
}

void loop()
{
    if (ep2.read_pkt())
    {
        if (ep2.get_pkt_type() == CRSF_FRAMETYPE_RC_CHANNELS)
        {
            ep2.get_rc_channels(&channels, true);
            Serial.print(channels.ch[0]);
            Serial.print("\t");
            Serial.print(channels.ch[1]);
            Serial.print("\t");
            Serial.print(channels.ch[2]);
            Serial.print("\t");
            Serial.print(channels.ch[3]);
            Serial.print("\t");
            Serial.print(channels.ch[4]);
            Serial.print("\t");
            Serial.print(channels.ch[5]);
            Serial.print("\t");
            Serial.print(channels.ch[6]);
            Serial.print("\t");
            Serial.println(channels.ch[7]);
        }
        else
        {
            Serial.println("  UNKNOWN PACKAGE");
            ep2.get_default_packet(&def_pkt);
            for (uint8_t i; i < (*def_pkt.len + 2); i++)
            {
                Serial.print(def_pkt.packet[i], HEX);
                if (def_pkt.packet[i] < 0x10)
                {
                    Serial.print("0");
                }
                Serial.print(" ");
            }
            Serial.println("\n  END UNKNOWN PACKAGE");
        }
    }
    delay(2);
}