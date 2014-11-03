#include <Wire.h>

#define DCC 0x01

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  //Serial.println("Type serial to start");
  //while (Serial.read() <= 0) {}
  
  //set_train(0, 3, true, 10);
}

void loop() {
    
  set_train(0, 3, true, 10);
  delay(1000);  
}


void set_train(uint8_t track, uint8_t train, bool forwards, uint8_t speed) {

  send(make_cmd(track, false, 1), train, make_speed(forwards, speed));

}

uint8_t make_cmd(uint8_t track, bool cancel, uint8_t repeat) {

return (track && 0b111) | cancel << 3 | (repeat && 0x1111) << 4;

}


uint8_t make_speed(bool forwards, uint8_t speed) {

  //DCC bit packs speed in a very strange way

  if(speed > 0) {

    speed += 3;

  }

  uint8_t c = speed & 0b00000001;

  speed &= 0b00011111;

  speed >>= 1;

  speed |= (c<<4);

  if(forwards) {

    return 0b01100000 | speed;

  } else {

    return 0b01000000 | speed;

  }

}


void send(uint8_t rawcmd, uint8_t address, uint8_t dcc) {

  Wire.beginTransmission(DCC);

  Serial.print(" Sending: ");

  Wire.write(rawcmd);

  Serial.print(rawcmd, BIN);

  Serial.print(", ");

  Wire.write(address);

  Serial.print(address, BIN);

  Serial.print(", ");

  Wire.write(dcc);

  Serial.print(dcc, BIN);

  Serial.print(", ");

  Wire.write(rawcmd ^ address ^ dcc);

  Serial.println(rawcmd ^ address ^ dcc, BIN);

  switch(Wire.endTransmission()) {

    case 0: //success

      break;

    case 1:

      Serial.println(" I2C send failed: too long");

      break;

    case 2:

      Serial.println(" I2C send failed: address NACK");

      break;

    case 3:

      Serial.println(" I2C send failed: data NACK");

      break;

    default:

      Serial.println(" I2C send failed: other");

  }

  uint8_t ret = Wire.requestFrom(DCC, 2);

  if(ret != 2) {

      Serial.print(" I2C didn't reply with two status bytes, bytes read:");

      Serial.println(ret);

  } else {

    uint8_t a = Wire.read();

    uint8_t b = Wire.read();

    if(a != ~b) {

      Serial.print(" Slave reply failed checksum:");

      Serial.print(a);

      Serial.print("!=");

      Serial.println(~b);

    } else {

      switch(a) {

        case 0:

          Serial.println("Packet accepted");

          break;

        case 1:

          Serial.println(" Packet was malformed.");

          break;

        case 2:

          Serial.println(" Packet failed checksum.");

          break;

        case 3:

          Serial.println(" Invalid track.");

          break;

        default:

          Serial.println(" Other Error.");

      }

    }

  }

}
