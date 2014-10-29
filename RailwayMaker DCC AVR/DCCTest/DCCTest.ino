/*
 * DCC control example
 *
 * Train id 3 shunts back and forth at 1Hz.
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

#include <DCC.h>

Train *train;

int main(void) {
    // Don't init USB its interupts are too slow
	init();
	setup();
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
	return 0;
}


void setup() {
  Track *track0 = DCC.add_track(5, 10, 9);
  train = (Train*)track0->add_decoder(new Train(3));
  train->set_speed(5);
  DCC.begin();
};

void loop() {
  train->reverse();
  DCC.first_track->current_decoder = NULL; //force next packet
  delay(500);
};
