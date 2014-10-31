/*
 * DCC over I2C
 *
 * Send DCC packets to Proxy via I2C
 *
 * © 2014 b@Zi.iS, Gary Fletcher
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
#include <SPI.h>
#include <SdFat.h>

SdFat sd;
#define SD_SELECT 8

#include <IniFile.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_PWMServoDriver.h>




#define PIN 4 // NEOPIXEL Pin num
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

int RECV_PIN = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;

// NOTE 0x27 ADDRESS RESERVED!
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // 2 Line
LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    // 4 Line


#define DCC 0x01

Adafruit_MCP23017 mcp[2];

bool forwards = false;
int ir_speed = 0;
int ir_train = 3;

typedef struct
{
    //int             address;                // DCC Address to respond to 0=n/a
    bool              type;                   // Type 1=toggle, 0=button/input
    bool              defaultState;           // Default state of pin 1=on, 0=off
    //int               outputMuxPin;           // Mux output pin to drive        
    //int               outputArduinoPin;       // Arduino output pin to drive (digital)
    int               servoToToggle;          // PWM Servo to toggle to on/off position
    //boolean           isFlasher;              // true=flash output, false=no time, no flash.
    //int               durationMilli;          // Milliseconds to leave output on for.  0 means don't auto off
    
    //int     onMilli;                // Used internally for timing if flasher
    //int     offMilli;               // 
} muxIO;
muxIO muxIOConfig[32];

typedef struct
{
    int               address;                // DCC Address to respond to 0=n/a
    int               defaultPos;             // Default position
    int               offPos;                 // Default position
    int               onPos;                  // Default position    
    byte              sweepSpeed;             // Speed servo moves
    boolean           isConstantSweep;        // Allows the servo to constantly sweep back and forth rather than toggle
    
} servoPWM;
//servoPWM servoConfig[32]; 

void setup() {
  
  
  Serial.begin(9600);
  
  Serial.println("Type any character to start");
  while (Serial.read() <= 0) {}
  
  Wire.begin();
  delay(100);
  
  // Configure all of the SPI select pins as outputs and make SPI
  // devices inactive, otherwise the earlier init routines may fail
  // for devices which have not yet been configured.
  pinMode(SD_SELECT, OUTPUT);
  digitalWrite(SD_SELECT, HIGH); // disable SD card
  
  // LCD
  //lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight  
  lcd.clear();
  lcd.backlight(); // finish with backlight on  

  // CONFIG
  const size_t bufferLen = 80;
  char buffer[bufferLen];

  const char *filename = "/config.txt";
  Serial.begin(9600);
  SPI.begin();
  if (!sd.begin(SD_SELECT))
    while (1)
      Serial.println("SD failed");
  
  IniFile ini(filename);
  if (!ini.open()) {
    Serial.print("config missing");
    // Cannot do anything else
    while (1)
      ;
  }

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print(" not valid: ");
    //printErrorMessage(ini.getError());
    // Cannot do anything else
    //while (1)
    //  ;
  }
  
  bool blnVal; 
  bool found;
  int m = 1; // mux to 32
  String setting;
  char buf[30];
  
  for(int y=0; y<2; y++) 
  {
    for(int x=0; x<16; x++) 
    {
      memcpy(buf, "type_", 5);
      itoa(m, &buf[5], 10);      
      
      blnVal = false;
      found = false;
      Serial.print(buf);
      lcd.setCursor(0, 1);
      lcd.print(buf);

      found = ini.getValue("mux", buf, buffer, bufferLen, blnVal);
      if (found)
      {
        muxIOConfig[m].type = blnVal;
        Serial.print("=");
        Serial.println(blnVal);
        lcd.setCursor(0, 2);
        lcd.print(blnVal);
      }


      
      //look for defaultState_
      memcpy(buf, "state_", 6);
      itoa(m, &buf[6], 10);      
      
      blnVal = false;
      found = false;
      Serial.print(buf);
      lcd.setCursor(0, 1);
      lcd.print(buf);

      found = ini.getValue("mux", buf, buffer, bufferLen, blnVal);
      if (found)
      {
        muxIOConfig[m].defaultState = blnVal;
        Serial.print("=");
        Serial.println(blnVal);
        lcd.setCursor(0, 2);
        lcd.print(blnVal);
      }


      
      m=m+1;
    }
  }
  
  Serial.print("???=");
  Serial.println(muxIOConfig[19].defaultState);
  Serial.println(muxIOConfig[19].defaultState ? "y" : "n");
  
  // Neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // IR
  irrecv.enableIRIn(); // Start the receiver
    
  // DEBUG
  scanI2C();

  delay(200);
  lcd.clear();
  
  //MUX
  setMux();
  
//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
   
  // Test Lights
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 15); // Blue
  //colorWipe(strip.Color(255, 255, 51), 50); // White White ???
 
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
  
}

int LoopCount=0;
void loop() {
  
  if (irrecv.decode(&results)) {

    //Serial.println(results.value, HEX);

    if(results.value == 0x77E110E4)
    {
      Serial.println("forward");
      ir_speed = ir_speed--;
      set_train(0, ir_train, true, ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );
    }
    if(results.value == 0x77E1E0E4)
    {
      //Serial.println("backward");
      ir_speed = ir_speed++;
      set_train(0, ir_train, ((ir_speed == 0) ? true : false), ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );
    }
    else if(results.value == 0x77E1D0E4)
    {
      //Serial.println("up");
      ir_train = ir_train++;
    }  
    else if(results.value == 0x77E1B0E4)
    {
      //Serial.println("down");
      ir_train = ir_train--;
    } 
    else if(results.value == 0x77E17AE4)
    {
      //Serial.println("play"); // can be the same as middle...
      set_train(0, 3, true, 0);
    }   
    else if(results.value == 0x77E120E4)
    {
      //Serial.println("middle");
 
      ir_speed = 0;
      set_train(0, ir_train, false, ir_speed );
    }  
    else if(results.value == 0x77E140E4)
    {
      //Serial.println("menu");
    }
    irrecv.resume(); // Receive the next value
  }

  muxStatus();
  setMux();
}

void setMux()
{
  int m = 1; // mux to 32
  for(int y=0; y<2; y++) 
  {
    mcp[y] = Adafruit_MCP23017();
    mcp[y].begin(y ? 3 : 7);
    for(int x=0; x<16; x++) 
    {
      if(muxIOConfig[m].type)
      {
        mcp[y].pinMode(x, INPUT);
        mcp[y].pullUp(x, HIGH);
      }
      else
      {
          if(muxIOConfig[m].defaultState)
          {
            mcp[y].digitalWrite(x, HIGH);
          }
          else
          {
            mcp[y].digitalWrite(x, LOW);
          }
      }
      m=m+1;
    }
  }
}

void muxStatus()
{
  for(int y=0; y<2; y++) 
  {
    for(int x=0; x<16; x++) 
    {
      lcd.setCursor(x, y+2);
      lcd.print(mcp[y].digitalRead(x));
    }
  }
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

  //Serial.print(" Sending: ");

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
/*
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
*/
  }

  uint8_t ret = Wire.requestFrom(DCC, 2);

  if(ret != 2) {

      Serial.print(" I2C err:");

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
/*
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
*/
    }

  }

}

// Fill the dots one after the other with a color
/*
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}
*/
void scanI2C()
{
  byte error, address;
  int nDevices;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Scanning...");
  delay(100);
  
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      lcd.setCursor(0,1);
      lcd.print("found 0x");
      lcd.setCursor(8,1);
      if (address<16) 
        lcd.print("0");
      lcd.print(address,HEX);
      Serial.println(address,HEX);
      //lcd.print("  !");
      delay(200);
      nDevices++;
    }
    else if (error==4) 
    {
      lcd.setCursor(0,1);
      lcd.print("error 0x");
      if (address<16) 
        lcd.print("0");
      lcd.print(address,HEX);
      delay(1000);
    }    
  }
  
  lcd.clear();
  lcd.setCursor(0,0);
  if (nDevices == 0)
    lcd.println("No I2C"); // No I2C devices

  //delay(500);           // wait 5 seconds for next scan
}



