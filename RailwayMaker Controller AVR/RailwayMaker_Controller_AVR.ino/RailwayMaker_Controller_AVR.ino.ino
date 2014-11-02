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
//#include <SPI.h>
#include <SdFat.h>
//#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_PWMServoDriver.h>


const uint8_t chipSelect = 8;
SdFat sd;

// create a serial stream
//ArduinoOutStream cout(Serial);


#define PIN 4 // NEOPIXEL Pin num
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
//Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x70);
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


IRrecv irrecv(12);
decode_results results;

// NOTE 0x27 ADDRESS RESERVED!
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // 2 Line
LiquidCrystal_I2C lcd(0x20, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);    // 4 Line


#define DCC 0x01

Adafruit_MCP23017 mcp[2];

typedef struct
{
    //bool              enabled = 0;               // if active or not   
    bool              defaultState = 0;          // Default state of pin 1=on, 0=off
    bool              currentState = 0;          // Default state of pin 1=on, 0=off
    byte              outputIO = 49;                // > 0 Mux output pin to drive, < 0 servo to toggle
    byte              servoMin = 0;              // 
    byte              servoMax = 0;              // 
    //bool              servoSweep = 0;            // if the servo should constantly sweep
    // Seconds to leave output on for.  
    // -1 = input button, 0 means don't auto off, > 0 is a toggle switch, < -1 is a on for once period
    int               durationSeconds;        
    
    unsigned long     onMilli = 0;            // Used internally for timing if flasher
} muxIO;
muxIO muxIOConfig[32];

void setup() {
  
  // Neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // LCD
  //lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight  
  lcd.clear();
  lcd.backlight(); // finish with backlight on  
  
  Serial.begin(9600);
  
  lcd.print("Type");
  while (Serial.read() <= 0) {}
  
  Wire.begin();
  delay(100);
  
  // Configure all of the SPI select pins as outputs and make SPI
  // devices inactive, otherwise the earlier init routines may fail
  // for devices which have not yet been configured.
  //lcd.setCursor(0,0);
  //lcd.print("Loading...");
  delay(400);  // catch Due reset problem

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();

  // run the example
  getSDline();
  Serial.println("h3");
  //cout << "\nDone!\n";
  
  // IR
  irrecv.enableIRIn(); // Start the receiver
  
  Serial.println("h4");
  // DEBUG
  //scanI2C();

  delay(200);
  //lcd.clear();
  Serial.println("h5");
    
  //MUX
  //setMux();
  
//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
   
  // Test Lights
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 15); // Blue
  //colorWipe(strip.Color(255, 255, 51), 50); // White White ???
 
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  //pwm2.begin();
  //pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm1.setPWM(1, 0, pulselen);
  }
  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm1.setPWM(1, 0, pulselen);
  }
  delay(500);  
  
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
  
  
  set_train(0, 3, true, 0);
  
}

void loop() {
  
  colorWipe(strip.Color(255, 0, 0), 1);
  colorWipe(strip.Color(255, 255, 0), 1);
  
  if (irrecv.decode(&results)) {

    //Serial.println(results.value, HEX);

    if(results.value == 0x77E110E4)
    {
      //Serial.println("forward");
      //ir_speed = ir_speed--;
      //set_train(0, ir_train, true, ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );
    }
    if(results.value == 0x77E1E0E4)
    {
      //Serial.println("backward");
      //ir_speed = ir_speed++;
      //set_train(0, ir_train, ((ir_speed == 0) ? true : false), ((ir_speed < 0) ? (ir_speed * -1) : ir_speed)  );
    }
    else if(results.value == 0x77E1D0E4)
    {
      //Serial.println("up");
      //ir_train = ir_train++;
    }  
    else if(results.value == 0x77E1B0E4)
    {
      //Serial.println("down");
      //ir_train = ir_train--;
    } 
    else if(results.value == 0x77E17AE4)
    {
      //Serial.println("play"); // can be the same as middle...
      //set_train(0, 3, true, 0);
    }   
    else if(results.value == 0x77E120E4)
    {
      //Serial.println("middle");
 
      //ir_speed = 0;
      //set_train(0, ir_train, false, ir_speed );
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
      //bool state = mcp[y].digitalRead(x);
      
      // BUTTON
      if(muxIOConfig[m].durationSeconds == -1)
      {
        if(muxIOConfig[m].onMilli == 0)
        {
          mcp[y].pinMode(x, INPUT);
          mcp[y].pullUp(x, HIGH);
          //muxIOConfig[m].onMilli = 1;
        }
        // MOVE SERVO TO POS
        
        if(muxIOConfig[m].outputIO > 32)
        {
          
          //Serial.print("i=");
          //Serial.println(m);
          //Serial.print("s=");
          //Serial.println(muxIOConfig[m].outputIO-32);
                            
          if(mcp[y].digitalRead(x))
          {
            //Serial.println("x");
            pwm1.setPWM(1, 0, 150);
          }
          else
          {
            //Serial.println("y");
            pwm1.setPWM(1, 0, 550);
          }
          
        }
        
        
      }
      else // TOGGLE
      {
          mcp[y].pinMode(x, OUTPUT);
          
          // DOES NOT toggle && not initialised 
          if(muxIOConfig[m].durationSeconds == 0) 
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
          else
          {
            mcp[y].digitalWrite(x, mcp[y].digitalRead(x));
          }
          // DOES toggle && not initialised 
          if(muxIOConfig[m].durationSeconds > 0) 
          {            
            
            //Serial.println(m);
            //Serial.println(muxIOConfig[m].onMilli);
            
            if(muxIOConfig[m].onMilli == 0) // initilise
            {
              //Serial.println("i");
              muxIOConfig[m].onMilli = 1; // not zero
              muxIOConfig[m].currentState = (muxIOConfig[m].defaultState ? LOW : HIGH);
            }
            else if(millis() > (muxIOConfig[m].onMilli+(muxIOConfig[m].durationSeconds*1000))) // Expired
            {
              //Serial.println("e");
              mcp[y].digitalWrite(x, (muxIOConfig[m].currentState ? LOW : HIGH)); // toggle // (muxIOConfig[m].defaultState ? LOW : HIGH)
              muxIOConfig[m].currentState = (muxIOConfig[m].currentState ? LOW : HIGH);
              muxIOConfig[m].onMilli = millis(); // reset
            }
            else // stick with curent value
            {
              mcp[y].digitalWrite(x,(muxIOConfig[m].currentState ? HIGH : LOW));
            }
            
          }
      }
      m=m+1;
    }
  }
}

void muxStatus()
{
  int m =1;
  for(int y=0; y<2; y++) 
  {
    for(int x=0; x<16; x++) 
    {
      Serial.println("h6");
      lcd.setCursor(x, y+2);
        Serial.println("h7");
      lcd.print(mcp[y].digitalRead(x));      
      m=m+1;
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

  //Serial.print(rawcmd, BIN);

  //Serial.print(", ");

  Wire.write(address);

  //Serial.print(address, BIN);

  //Serial.print(", ");

  Wire.write(dcc);

  //Serial.print(dcc, BIN);

  //Serial.print(", ");

  Wire.write(rawcmd ^ address ^ dcc);

  //Serial.println(rawcmd ^ address ^ dcc, BIN);
/*
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

      //Serial.print(" I2C err:");

      //Serial.println(ret);

  } else {

    uint8_t a = Wire.read();

    uint8_t b = Wire.read();

    if(a != ~b) {

      //Serial.print(" Slave reply failed checksum:");

      //Serial.print(a);

      //Serial.print("!=");

      //Serial.println(~b);

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
*/
}

// Fill the dots one after the other with a color

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}
/*
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
*/

#define DEBUGLOADCONFIG

void getSDline() {
  
  String key;
  String val;
  
  const byte line_buffer_size = 16;
  char buffer[line_buffer_size];
  ifstream sdin("config.txt");
  int line_number = 0;

  while (sdin.getline(buffer, line_buffer_size, '\n') || sdin.gcount()) {
    int count = sdin.gcount();
    if (sdin.fail()) {
      //cout << "Partial long line";
      Serial.println("Line fail");
      sdin.clear(sdin.rdstate() & ~ios_base::failbit);
    } else if (sdin.eof()) {
      //cout << "Partial final line";  // sdin.fail() is false
    } else {
      count--;  // Don’t include newline in count
      //cout << "Line " << ++line_number;
    }
    //cout << " (" << count << " chars): " << buffer << endl;
    String s = String(buffer);
    if(s.indexOf("=")>0)
    {
        #ifdef DEBUGLOADCONFIG
          Serial.print( s.substring(0,s.indexOf("=")-3) );
          Serial.print("^");
          Serial.print( (s.substring(s.substring(0,s.indexOf("=")-3).length(),s.indexOf("=")-1)) );
          Serial.print("^");
          Serial.println( s.substring(s.indexOf("=")+2,s.length()) );
        #endif
        
        byte m = (s.substring(s.substring(0,s.indexOf("=")-3).length(),s.indexOf("=")-1)).toInt();
        int v = s.substring(s.indexOf("=")+2,s.length()).toInt();
      
        if(s.substring(0,s.indexOf("=")-3) == "state")
          muxIOConfig[m].defaultState = v; // state_
        
        if(s.substring(0,s.indexOf("=")-3) == "sec")
          muxIOConfig[m].durationSeconds = v; 
        
        if(s.substring(0,s.indexOf("=")-3) == "out")
          muxIOConfig[m].outputIO = v;
         
        //if(s.substring(0,s.indexOf("=")-3) == "min")
        //  muxIOConfig[m].servoMin = s.substring(s.indexOf("=")+2,s.length()).toInt(); 
        //if(s.substring(0,s.indexOf("=")-3) == "max")
        //  muxIOConfig[m].servoMax = s.substring(s.indexOf("=")+2,s.length()).toInt();
    }   
    Serial.println("h1");
  }
    Serial.println("h2");
}

