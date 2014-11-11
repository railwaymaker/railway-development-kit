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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _DCC_TRAIN_H
#define _DCC_TRAIN_H

class Train: public Decoder {
        uint8_t             mypacket[4];
        uint8_t             nextcmd;
        uint8_t             nextchk;
    public:
        Train(uint8_t);
        void                set_speed(uint8_t);
        uint8_t             get_speed();
        void                set_direction(bool);
        bool                get_direction();
        void                reverse();
        virtual bool        next_packet();
        virtual void        debug_packet();
};

#endif
