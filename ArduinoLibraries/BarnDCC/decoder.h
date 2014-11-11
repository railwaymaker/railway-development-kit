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

#ifndef _DCC_DECODER_H
#define _DCC_DECODER_H

#define CCC_DECODER         B00000000
#define CCC_ADVANCED        B00100000
#define CCC_REVERSE         B01000000
#define CCC_FORWARD         B01100000
#define CCC_FG1             B10000000
#define CCC_FG2             B10100000
#define CCC_CONFIG          B11100000

class Decoder {
    public:
        Decoder(uint8_t);
        uint8_t             add;
        Decoder *           next;
        uint8_t             (*packet)[4];
        virtual uint8_t     get_packet_length();
        virtual bool        next_packet(); //returns true if the are no more packets
        virtual void        debug_packet();
        virtual void        debug_wave();
};

#endif
