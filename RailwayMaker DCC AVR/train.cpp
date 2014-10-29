/*
 * DCC Train
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

Train::Train(uint8_t address) : Decoder(address) {
    mypacket[2] = address;
    mypacket[1] = 0;
    mypacket[0] = 0;
    nextcmd = CCC_FORWARD;
    nextchk = 0;
    packet = &mypacket;
}

void Train::set_speed(uint8_t s) {
    uint8_t c;
    if(s>0) {
        s += 3;
    }
    c = s & B00000001;
    s &= B00011111;
    s >>= 1;
    s |= (c<<4);
    nextcmd = (nextcmd & B11100000) | s;
    nextchk = add ^ nextcmd;
}

uint8_t Train::get_speed() {
    uint8_t s,c;
    c = nextcmd & B00010000;
    s = nextcmd & B00001111;
    s <<= 1;
    s |= (c>>4);
    s -= 3;
    return s;
}

void Train::set_direction(bool forward) {
    nextcmd = (nextcmd & B00011111) | (forward ? CCC_FORWARD : CCC_REVERSE);
    nextchk = add ^ nextcmd;
}

bool Train::get_direction() {
    return (nextcmd & B11100000) == CCC_FORWARD;
}

void Train::reverse() {
    set_direction(!get_direction());
}

bool Train::next_packet() {
    mypacket[1] = nextcmd;
    mypacket[0] = nextchk;
    return false;
}

void Train::debug_packet() {
    Serial.print(F("Train Address:"));
    Serial.print(add);
    if(get_direction()) {
        Serial.print(F(" Forwards"));
    } else {
        Serial.print(F(" Backwards"));
    }
    Serial.print(F(" Speed:"));
    Serial.println(get_speed());
    Serial.println(F("111111111111110AAAAAAAA001DCSSSS0CCCCCCCC"));
    Decoder::debug_packet();
}
