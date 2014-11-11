/*
 * DCC control
 *
 * © 2014 b@Zi.iS
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MAINTAINABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "DCC.h"

Decoder::Decoder(uint8_t address) {
    next = NULL;
    add = address;
    packet = &DCC.idle;
}

uint8_t Decoder::get_packet_length() {
    return 27;
}

bool Decoder::next_packet() {
    return false;
}

void Decoder::debug_packet() {
    if(get_packet_length()>0) {
        Serial.print(F("11111111111111"));
        for(int8_t i=get_packet_length()-1; i>=0; i--) {
            Serial.print((bool)((*packet)[i / 9] & (1 << (i % 9))));
        }
        Serial.println();
    } else {
        Serial.println(F("No packet"));
    }
}

void Decoder::debug_wave() {
    if(get_packet_length()>0) {
        uint16_t time = 1624;
        Serial.print(F("~_~_~_~_~_~_~_~_~_~_~_~_~_~_"));
        for(int8_t i=get_packet_length()-1; i>=0; i--) {
            if((*packet)[i / 9] & (1 << (i % 9))) {
                Serial.print("~_");
                time += 58*2;
            } else {
                Serial.print("~~__");
                time += 58*4;
            }
        }
        Serial.println();
        Serial.print(time);
        Serial.println("us");
    }
}
