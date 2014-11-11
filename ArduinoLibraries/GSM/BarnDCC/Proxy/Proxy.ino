/*
 * DCC Proxy
 *
 * Proxy I2C and UART commands to the DCC.
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

#include <Wire.h>
#include <DCC.h>

#define ERR_OK           0x00
#define ERR_MALFORMED    0x01
#define ERR_CHECKSUM     0x02
#define ERR_NOTRACK      0x03

#define REPEAT_FOREVER   0x00
#define REPEAT_CANCEL    B00001111

struct CMD { // RRRRCTTT
  uint8_t track:3;
  bool cancel:1;
  bool repeat:4;
};

class Raw : public Decoder {
  uint8_t repeat;
  public:
    uint8_t             mypacket[4];
    uint8_t             cmd;
    Raw(uint8_t repeat, uint8_t address, uint8_t cmd) : Decoder(address) {
      this->cmd = cmd;
      mypacket[2] = address;
      mypacket[1] = cmd;
      mypacket[0] = address ^ cmd;
      packet = &mypacket;
      this->repeat = repeat + 1;
    }
    virtual bool next_packet() {
      if (repeat==1) {
        return true;
      }
      if (repeat>1) {
        repeat--;
      }
      return false;
    }
    void cancel() {
      repeat = 1;
    }
};

uint8_t error = ERR_OK;

void setup() {
 
  Serial.begin(115200);
  
  Wire.begin(0x01);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  DCC.add_track(5, 10, 9);
  DCC.begin();
}

void loop() {
  
  if(error != ERR_OK) {
    Serial.print("Error:");
    Serial.println(error);
  }
  
  //Debuging output
  Track *track = DCC.first_track; 
  while(track != NULL) {
    
      Serial.print("Track:");
      Serial.println((unsigned int)track, HEX);
      
      Decoder *dec = track->first_decoder;
      while(dec != NULL) {
        Serial.print("CMD:");
        dec->debug_packet();
        dec->debug_wave();
        dec = dec->next;
      }
      
      track = track->next;
  }
  
  // Serial input
  // TRACK_NO:TRAIN_ADDRESS=SPEED, i.e. 0:1=1 is slow forward 0:1:-31 is fast reverese
  if (Serial.available()) {
    uint32_t track = Serial.parseInt();
    if(Serial.read()==':') {
       uint32_t train = Serial.parseInt();      
      if(Serial.read()=='=') {
        int32_t  speed = Serial.parseInt();
        Serial.print("Track ");
        Serial.print(track);
        Serial.print(" Train ");
        Serial.print(train);
        if(speed >= 0) {
          Serial.print(" Forwards at ");
          Serial.println(speed & B11111);
          speed = CCC_FORWARD | (speed & B11111);
        } else {
          Serial.print(" Backwards at ");
          Serial.println(-speed & B11111);
          speed = CCC_REVERSE  | (-speed & B11111);
        }
        
        parse(B00001000 | track, train, speed, (B00001000 | track) ^ train ^ speed);
      }
    }
  }
  
  delay(1000);
  
}

void receiveEvent(int howMany) {
  if (howMany != 4) {
    Serial.print("ERROR:Malformed");
    error = ERR_MALFORMED;
    return;
  }
  return parse(Wire.read(), Wire.read(), Wire.read(), Wire.read());
}

Track* get_track(uint8_t idx) {
  Track* tmp = DCC.first_track;
  while(tmp != NULL) {
    if(idx == 0) {
      return tmp;
    }
    tmp = tmp->next;
    idx--;
  }
  return NULL;
}

void parse(uint8_t rawcmd, uint8_t address, uint8_t dcc, uint8_t checksum) {
  if ((rawcmd ^ address ^ dcc) != checksum) {
    error = ERR_CHECKSUM;
    return;
  }
  error = ERR_OK;
  CMD cmd = *(CMD *)&rawcmd;
  Track* track = get_track(cmd.track);
  if(track == NULL) {
    error = ERR_NOTRACK;
    return;
  }
  if (cmd.repeat == REPEAT_CANCEL) { //Cancel given command
    Decoder* tmp = track->first_decoder;
    while (tmp != NULL) {
      if(tmp->add == address && ((Raw*)tmp)->cmd == dcc) {
        ((Raw*)tmp)->cancel();
      }
      tmp = tmp->next;
    }
    return;
  }
  if (cmd.cancel) { //Cancel all other commands for address
    Decoder* tmp = track->first_decoder;
    while (tmp != NULL) {
      if(tmp->add == address) {
        ((Raw*)tmp)->cancel();
      }
      tmp = tmp->next;
    }
  }
  track->add_decoder(new Raw(cmd.repeat, address, dcc));  
}

void requestEvent() {
  Wire.write(error);
  Wire.write(~error);
  error = ERR_OK;
}
