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
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_PWMServoDriver.h>


// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 8; 

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

int sensorPin = A5;
int sensorValue = 0;
int sensorMin = 0;        // minimum sensor value
int sensorMax = 31;           // maximum sensor value


typedef struct
{
    int               address;                // DCC Address to respond to 0=n/a
    byte              type;                   // Type 1=toggle, 0=button/input
    byte              defaultState;           // Default state of pin 1=on, 0=off
    int               outputMuxPin;           // Mux output pin to drive        
    int               outputArduinoPin;       // Arduino output pin to drive (digital)
    int               servoToToggle;          // PWM Servo to toggle to on/off position
    boolean           isFlasher;              // true=flash output, false=no time, no flash.
    int               durationMilli;          // Milliseconds to leave output on for.  0 means don't auto off
    
    unsigned long     onMilli;                // Used internally for timing if flasher
    unsigned long     offMilli;               // 
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
servoPWM servoConfig[32]; 

void setup() {
  
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  Wire.begin();
  Serial.println("loading");
  
  
  colorWipe(strip.Color(255, 255, 51), 5);
  
  // SD pin 10 must be set as output
  pinMode(10, OUTPUT);     
  
  //MUX
  setMux();

  // Neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // IR
  irrecv.enableIRIn(); // Start the receiver
  
  // LCD
  //lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight

// ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 2; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  
  
  // DEBUG
  scanI2C();

  lcd.setCursor(0,0);
  lcd.print("Initializing SD");
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    lcd.setCursor(0,1);
    lcd.print("sd failed");
    return;
  } else {
   lcd.setCursor(0,1);
   lcd.print("sd present"); 
  }
  
  delay(1000);
  lcd.clear();

//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("T");
  lcd.print(ir_train);
  
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("RailwayMaker.com");
  //delay(8000);

  set_train(0, ir_train, false, ir_speed );

  lcd.setCursor(4,0);
  lcd.print( "-stop-" );
  
  //set_train(0, 3, forwards, 10);
  
  // Test Lights
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 15); // Blue
  colorWipe(strip.Color(255, 255, 51), 50); // White White ???
 
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
  
  

  
}

int LoopCount=0;
void loop() {
  
  /*
  sensorValue = analogRead(sensorPin);
  sensorValue = map(sensorValue, 0, 1023, -5, 5);
  
  if(sensorValue > 0)
  {
    forwards = true;
  }
  else
  {
    forwards = false;
    sensorValue = sensorValue * -1;
  }
  
  //forwards = !forwards;
  //set_train(0, 3, forwards, 10);
  //delay(5000);
  lcd.setCursor(0,0);
  lcd.print("                            ");
  lcd.setCursor(0,0);
  lcd.print(sensorValue);
  set_train(0, 3, forwards, sensorValue);
  delay(50);
  */
  
  if (irrecv.decode(&results)) {

    //Serial.println(results.value, HEX);

    if(results.value == 0x77E110E4)
    {
      Serial.println("forward");
      ir_speed = ir_speed--;
      set_train(0, ir_train, true, ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );

      lcd.setCursor(0,0);
      lcd.print("T");
      lcd.setCursor(1,0);
      lcd.print(ir_train);
      
      
      lcd.setCursor(4,0);
      lcd.print(  ((ir_speed == 0) ? "-stop-" :  ((ir_speed < 0) ? "<< " : ">> " )) );
      
      if(ir_speed != 0)
      {
        lcd.setCursor(7,0);
        lcd.clear(); // clear
        lcd.setCursor(7,0);
        lcd.print(ir_speed);
      }
    }
    if(results.value == 0x77E1E0E4)
    {
      Serial.println("backward");
      ir_speed = ir_speed++;
      set_train(0, ir_train, ((ir_speed == 0) ? true : false), ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );

      lcd.setCursor(0,0);
      lcd.print("T");
      lcd.setCursor(1,0);
      lcd.print(ir_train);

      lcd.setCursor(4,0);
      lcd.print(  ((ir_speed == 0) ? "-stop-" :  ((ir_speed < 0) ? "<< " : ">> " )) );
      
      if(ir_speed != 0)
      {
        lcd.setCursor(7,0);
        lcd.clear(); // clear
        lcd.setCursor(7,0);
        lcd.print(ir_speed);
      }
    }
    else if(results.value == 0x77E1D0E4)
    {
      Serial.println("up");
      ir_train = ir_train++;
      lcd.setCursor(0,0);
      lcd.print("T");
      lcd.setCursor(1,0);
      lcd.print(ir_train);
    }  
    else if(results.value == 0x77E1B0E4)
    {
      Serial.println("down");
      ir_train = ir_train--;
      lcd.setCursor(0,0);
      lcd.print("T");
      lcd.setCursor(1,0);
      lcd.print(ir_train);
    } 
    else if(results.value == 0x77E17AE4)
    {
      Serial.println("play"); // can be the same as middle...
      set_train(0, 3, true, 0);
    }   
    else if(results.value == 0x77E120E4)
    {
      Serial.println("middle");
      
      ir_speed = 0;
      set_train(0, ir_train, false, ir_speed );

      lcd.setCursor(4,0);
      lcd.print( "-stop-" );
      
    }  
    else if(results.value == 0x77E140E4)
    {
      Serial.println("menu");
    }
    irrecv.resume(); // Receive the next value
  }

  muxStatus();
  setMux();
}

void setMux()
{
  for(int y=0; y<2; y++) 
  {
    mcp[y] = Adafruit_MCP23017();
    mcp[y].begin(y ? 3 : 7);
    for(int x=0; x<16; x++) 
    {
      mcp[y].pinMode(x, INPUT);
      mcp[y].pullUp(x, HIGH);
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

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

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
      delay(1000);
      nDevices++;
    }
    else if (error==4) 
    {
      lcd.setCursor(0,1);
      lcd.print("error at address 0x");
      if (address<16) 
        lcd.print("0");
      lcd.print(address,HEX);
      delay(1000);
    }    
  }
  
  lcd.clear();
  lcd.setCursor(0,0);
  if (nDevices == 0)
    lcd.print("No I2C devices found\n");
  else
    lcd.print("I2C done");

  delay(500);           // wait 5 seconds for next scan
}
