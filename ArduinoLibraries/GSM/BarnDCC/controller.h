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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DCC_CONTROLLER_H
#define _DCC_CONTROLLER_H

#include "decoder.h"

#ifdef TCCR2A
    #define TCCRXA          TCCR2A
    #define TCCRXB          TCCR2B
    #define TIMSKX          TIMSK2
    #define OCRXA           OCR2A
    #define TCNTX           TCNT2
    #define WGMX0           WGM20
    #define WGMX1           WGM21
    #define WGMX2           WGM22
    #define CSX0            CS20
    #define CSX1            CS21
    #define CSX2            CS22
    #define TOIEX           TOIE2
    #define RESOLUTION      256
#else
    #define TCCRXA          TCCR3A
    #define TCCRXB          TCCR3B
    #define TIMSKX          TIMSK3
    #define OCRXA           OCR3A
    #define TCNTX           TCNT3
    #define WGMX0           WGM30
    #define WGMX1           WGM31
    #define WGMX2           WGM32
    #define WGMX3           WGM33
    #define CSX0            CS30
    #define CSX1            CS31
    #define CSX2            CS32
    #define TOIEX           TOIE3
    #define RESOLUTION      65536
#endif

class Track {
        volatile uint8_t *  trig_port;
        uint8_t             trig_toggle;
        uint8_t             phase;
        uint8_t             transmit_idx;
        uint8_t             en;
        volatile uint8_t *  port;
        uint8_t             toggle;
        uint8_t             (*packet)[4];
        uint8_t             packet_length;
    public:
        Track(uint8_t, uint8_t, uint8_t);
        void                begin();
        inline void         next_packet();
        Decoder *           add_decoder(Decoder*);
        void                set_trigger(uint8_t);
        void                clear_trigger();
        inline void         tick();
        Track *             next;
        Decoder *           first_decoder;
        Decoder *           current_decoder;
};

class Controller {
    public:
        Controller();
        void begin(void);
        Track *             add_track(uint8_t, uint8_t, uint8_t);
        Track *             first_track;
        uint8_t             idle[4];
};

#endif
