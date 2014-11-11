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

Controller DCC;

Track *tmp;

#ifdef TCCR2A
ISR(TIMER2_OVF_vect) {
#else
ISR(TIMER3_OVF_vect) {
#endif
    tmp = DCC.first_track;
    while(tmp != NULL) {
        tmp->tick();
        tmp = tmp->next;
    }
}

Controller::Controller() {
    first_track = NULL;
    idle[0] = 0xFF;
    idle[1] = 0x00;
    idle[2] = 0xFF;
}

void Controller::begin() {
    //Trigger an interupt every 58µs
    uint8_t clockSelectBits;
    int8_t  oldSREG;

    TCCRXA = _BV(WGMX0) | _BV(WGMX1) |_BV(WGMX2);
#ifdef WGMX3
    TCCRXB = _BV(WGMX3);        //mode 15
#else
    TCCRXB = 0;                 //mode 7
#endif

    long microseconds = 58;
    long cycles = (F_CPU / 2000000) * microseconds;
    if(cycles < RESOLUTION)              clockSelectBits = _BV(CSX0);              // /1
    else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CSX1);              // /8
    else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CSX1) | _BV(CSX0);  // /64
    else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CSX2);              // /256
    else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CSX2) | _BV(CSX0);  // /1024

    oldSREG = SREG;
    cli();
    OCRXA = cycles;
    SREG = oldSREG;

    TCCRXB &= ~(_BV(CSX0) | _BV(CSX1) | _BV(CSX2));
    TCCRXB |= clockSelectBits;

    TIMSKX |= _BV(TOIEX);

    Track *tmp = first_track;
    while(tmp != NULL) {
        tmp->begin();
        tmp = tmp->next;
    }
}

Track* Controller::add_track(uint8_t enable, uint8_t output0, uint8_t output1) {
    Track *buf = new Track(enable, output0, output1);
    buf->next = first_track;
    first_track = buf;
    return buf;
}

Track::Track(uint8_t enable, uint8_t output0, uint8_t output1) {
    volatile uint8_t *out0_port, *out1_port;
    uint8_t out0_bit, out1_bit;

    next = NULL;
    first_decoder = NULL;
    current_decoder = NULL;
    next_packet();

    phase = 0;
    transmit_idx = 0;
    out0_port = portOutputRegister(digitalPinToPort(output0));
    out0_bit = digitalPinToBitMask(output0);
    out1_port = portOutputRegister(digitalPinToPort(output1));
    out1_bit = digitalPinToBitMask(output1);
    en = enable;
    port = out0_port;
    toggle = out0_bit | out1_bit;
    digitalWrite(en, LOW);
    pinMode(en, OUTPUT);
    pinMode(output0, OUTPUT);
    pinMode(output1, OUTPUT);
    digitalWrite(output0, HIGH);
    digitalWrite(output1, LOW);

}

inline void Track::next_packet() {
    Decoder* loop = NULL;
    if(current_decoder == NULL) {
        current_decoder = first_decoder;
    } else {
        current_decoder = current_decoder->next;
    }
    while(true) {
        if(first_decoder == NULL) {
            break; //nodecoders
        }
        if(current_decoder == NULL) {
            current_decoder = first_decoder;
        }
        if(loop == NULL) {
            loop = current_decoder;
        } else if(loop == current_decoder) {
            break; //already tried every decoder
        }
        if (current_decoder->next_packet()) {
            //No more packets delete decoder
            if (current_decoder == first_decoder) {
                first_decoder = current_decoder->next;
                delete current_decoder;
                continue;
            } else {
                Decoder* tmp = first_decoder;
                while(tmp != NULL) {
                   if(tmp->next == current_decoder) {
                        tmp->next = current_decoder->next;
                        delete current_decoder;
                        current_decoder = tmp->next;
                        continue;
                    }
                }
            }
        }
        uint8_t length = current_decoder->get_packet_length();
        if(length == 0) {
            current_decoder = current_decoder->next;
            continue;
        } else {
            packet = current_decoder->packet;
            packet_length = length;
            return;
        }
    }
    packet = &DCC.idle;
    packet_length = 27;
}

Decoder* Track::add_decoder(Decoder* decoder) {
    decoder->next = first_decoder;
    first_decoder = decoder;
    return decoder;
}

void Track::begin() {
    next_packet();
    digitalWrite(en, HIGH);
}

inline void Track::tick() {
    if (phase & 1) {
        phase++;
    } else {
        PINB = toggle; //*port ^= toggle;
        if(transmit_idx >= packet_length || ((*packet)[transmit_idx / 9] & (1 << (transmit_idx % 9)))) {
            phase += 2;
        } else {
            phase++;
        }
    }
    if(phase == 4) {
        phase = 0;
        if(transmit_idx-- == 0) {
            transmit_idx = packet_length + 14;
            next_packet();
        }
    }
}
