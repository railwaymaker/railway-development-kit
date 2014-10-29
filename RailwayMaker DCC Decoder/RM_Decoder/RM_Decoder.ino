#include <DCC_Decoder.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

typedef struct
{
    int               address;                // Address to respond to
    byte              output;                 // State of output 1=on, 0=off
    int               outputPin;              // Arduino output pin to drive
    boolean           isDigital;              // true=digital, false=analog. If analog must also set analogValue field
    boolean           isFlasher;              // true=flash output, false=no time, no flash.
    byte              analogValue;            // Value to use with analog type.
    int               durationMilli;          // Milliseconds to leave output on for.  0 means don't auto off
    
    unsigned long     onMilli;                // Used internally for timing
    unsigned long     offMilli;               // 
} DCCAccessoryAddress;

DCCAccessoryAddress gAddresses[8];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Decoder Init 
//
void ConfigureDecoder()
{
    gAddresses[0].address = 714;
    gAddresses[0].output = 0;
    gAddresses[0].outputPin = 5;
    gAddresses[0].isDigital = false;
    gAddresses[0].isFlasher = false;
    gAddresses[0].analogValue = 250;
    gAddresses[0].durationMilli = 500;
    
    gAddresses[1].address = 715;
    gAddresses[1].output = 0;
    gAddresses[1].outputPin = 6;
    gAddresses[1].isDigital = true;
    gAddresses[1].isFlasher = false;
    gAddresses[1].analogValue = 0;
    gAddresses[1].durationMilli = 500;
        
    gAddresses[2].address = 814;
    gAddresses[2].output = 0;
    gAddresses[2].outputPin = 5;
    gAddresses[2].isDigital = false;
    gAddresses[2].isFlasher = true;
    gAddresses[2].analogValue = 250;
    gAddresses[2].durationMilli = 500;
    
    gAddresses[3].address = 815;
    gAddresses[3].output = 0;
    gAddresses[3].outputPin = 6;
    gAddresses[3].isDigital = true;
    gAddresses[3].isFlasher = true;
    gAddresses[3].analogValue = 0;
    gAddresses[3].durationMilli = 500;
    
    gAddresses[4].address = 914;
    gAddresses[4].output = 0;
    gAddresses[4].outputPin = 5;
    gAddresses[4].isDigital = false;
    gAddresses[4].isFlasher = false;
    gAddresses[4].analogValue = 250;
    gAddresses[4].durationMilli = 0;
    
    gAddresses[5].address = 915;
    gAddresses[5].output = 0;
    gAddresses[5].outputPin = 6;
    gAddresses[5].isDigital = true;
    gAddresses[5].isFlasher = false;
    gAddresses[5].analogValue = 0;
    gAddresses[5].durationMilli = 0;
    
    gAddresses[6].address = 0;
    gAddresses[6].output = 0;
    gAddresses[6].outputPin = 0;
    gAddresses[6].isDigital = false;
    gAddresses[6].isFlasher = false;
    gAddresses[6].analogValue = 0;
    gAddresses[6].durationMilli = 0;
    
    gAddresses[7].address = 0;
    gAddresses[7].output = 0;
    gAddresses[7].outputPin = 0;
    gAddresses[7].isDigital = false;
    gAddresses[7].isFlasher = false;
    gAddresses[7].analogValue = 0;
    gAddresses[7].durationMilli = 0;
    
        // Setup output pins
    for(int i=0; i<(int)(sizeof(gAddresses)/sizeof(gAddresses[0])); i++)
    {
        if( gAddresses[i].outputPin )
        {
            pinMode( gAddresses[i].outputPin, OUTPUT );
        }
        gAddresses[i].onMilli = 0;
        gAddresses[i].offMilli = 0;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Basic accessory packet handler 
//
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
        // Convert NMRA packet address format to human address
    address -= 1;
    address *= 4;
    address += 1;
    address += (data & 0x06) >> 1;
    
    boolean enable = (data & 0x01) ? 1 : 0;
    
    for(int i=0; i<(int)(sizeof(gAddresses)/sizeof(gAddresses[0])); i++)
    {
        if( address == gAddresses[i].address )
        {
            Serial.print("Basic addr: ");
            Serial.print(address,DEC);
            Serial.print("   activate: ");
            Serial.println(enable,DEC);
            
            if( enable )
            {
                gAddresses[i].output = 1;
                gAddresses[i].onMilli = millis();
                gAddresses[i].offMilli = 0;
            }else{
                gAddresses[i].output = 0;
                gAddresses[i].onMilli = 0;
                gAddresses[i].offMilli = millis();
            }
        }
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//******* IR Start *******************************************************************************


/*
 * IRremote: IRrecvDump - dump details of IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and other people at the original blog post)
 */

#include <IRremote.h>

int RECV_PIN = 12; // GF

IRrecv irrecv(RECV_PIN);

decode_results results;


// Dumps out the decode_results structure.
// Call this after IRrecv::decode()
// void * to work around compiler issue
//void dump(void *v) {
//  decode_results *results = (decode_results *)v
void dump(decode_results *results) {
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) {
    Serial.print("Unknown encoding: ");
  } 
  else if (results->decode_type == NEC) {
    Serial.print("Decoded NEC: ");
  } 
  else if (results->decode_type == SONY) {
    Serial.print("Decoded SONY: ");
  } 
  else if (results->decode_type == RC5) {
    Serial.print("Decoded RC5: ");
  } 
  else if (results->decode_type == RC6) {
    Serial.print("Decoded RC6: ");
  }
  else if (results->decode_type == PANASONIC) {	
    Serial.print("Decoded PANASONIC - Address: ");
    Serial.print(results->panasonicAddress,HEX);
    Serial.print(" Value: ");
  }
  else if (results->decode_type == JVC) {
     Serial.print("Decoded JVC: ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
    if ((i % 2) == 1) {
      Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
    } 
    else {
      Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
    }
    Serial.print(" ");
  }
  Serial.println("");
}

//******* IR End *******************************************************************************


//******* Servo Start *******************************************************************************

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41); // GF

//******* Servo Start *******************************************************************************


//******* SD Start *******************************************************************************

/*
  SD card test 
   
 This example shows how use the utility libraries on which the'
 SD library is based in order to get info about your SD card.
 Very useful for testing a card when you're not sure whether its working or not.
 	
 The circuit:
  * SD card attached to SPI bus as follows:
 ** MOSI - pin 11 on Arduino Uno/Duemilanove/Diecimila
 ** MISO - pin 12 on Arduino Uno/Duemilanove/Diecimila
 ** CLK - pin 13 on Arduino Uno/Duemilanove/Diecimila
 ** CS - depends on your SD card shield or module. 
 		Pin 4 used here for consistency with other Arduino examples

 
 created  28 Mar 2011
 by Limor Fried 
 modified 9 Apr 2012
 by Tom Igoe
 */
 // include the SD library:
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 8;   

//******* SD End *******************************************************************************



//******* MCP23017 Start *******************************************************************************

//#include <Wire.h>
#include "Adafruit_MCP23017.h"

// Basic pin reading and pullup test for the MCP23017 I/O expander
// public domain!

// Connect pin #12 of the expander to Analog 5 (i2c clock)
// Connect pin #13 of the expander to Analog 4 (i2c data)
// Connect pins #15, 16 and 17 of the expander to ground (address selection)
// Connect pin #9 of the expander to 5V (power)
// Connect pin #10 of the expander to ground (common ground)

// Input #0 is on pin 21 so connect a button or switch from there to ground

Adafruit_MCP23017 mcp;

//******* MCP23017 End *******************************************************************************



//******* NeoPixel Start *******************************************************************************
#include <Adafruit_NeoPixel.h>

#define PIN 4

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.
//******* NeoPixel End *******************************************************************************


void setup() {
  
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  //******* DCC Start *******************************************************************************
   DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
   ConfigureDecoder();
   DCC.SetupDecoder( 0x00, 0x00, kDCC_INTERRUPT );  
  //******* DCC Start *******************************************************************************
  
    
  //******* NeoPixel Start *******************************************************************************
  // NeoPixel
  strip.begin(); 
  strip.show(); // Initialize all pixels to 'off'
  //******* NeoPixel End *******************************************************************************
  
  //******* MCP23017 Start *******************************************************************************

  mcp.begin(0x27);      // use default address 0 // GF not standard

  mcp.pinMode(0, INPUT);
  mcp.pullUp(0, HIGH);  // turn on a 100K pullup internally

  //pinMode(13, OUTPUT);  // use the p13 LED as debugging
  //******* MCP23017 End *******************************************************************************

  //******* SD Start *******************************************************************************

  Serial.print("\nInitializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(10, OUTPUT);     // change this to 53 on a mega


  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    /return;
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    //return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);

  
  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  //******* SD End *******************************************************************************

  //******* Servo Start *******************************************************************************
  
  Serial.println("16 channel PWM test!");

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
 

  //******* Servo End *******************************************************************************

  //******* IR Start *******************************************************************************

  irrecv.enableIRIn(); // Start the receiver

  //******* IR End *******************************************************************************


}

void loop() {
  
  //******* NeoPixel Start *******************************************************************************
  // Some example procedures showing how to display to the pixels:
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 50); // Blue
  // Send a theater pixel chase in...
  //theaterChase(strip.Color(127, 127, 127), 50); // White
  //theaterChase(strip.Color(127,   0,   0), 50); // Red
  //theaterChase(strip.Color(  0,   0, 127), 50); // Blue

  //rainbow(20);
  //rainbowCycle(20);
  theaterChaseRainbow(50);
  //******* NeoPixel End *******************************************************************************
  
  //******* MCP23017 Start *******************************************************************************

  // The LED will 'echo' the button
  //digitalWrite(13, mcp.digitalRead(0));
  mcp.digitalRead(0);

  //******* MCP23017 End *******************************************************************************

  //******* Servo Start *******************************************************************************

  // Drive each PWM in a 'wave'
  for (uint16_t i=0; i<4096; i += 8) {
    for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
      pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
    }
  }
  
  //******* Servo End *******************************************************************************


  //******* IR Start *******************************************************************************

  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    dump(&results);
    irrecv.resume(); // Receive the next value
  }

  //******* IR End *******************************************************************************


  //******* DCC Start *******************************************************************************


    static int addr = 0;
    
        ////////////////////////////////////////////////////////////////
        // Loop DCC library
    DCC.loop();
    
        ////////////////////////////////////////////////////////////////
        // Bump to next address to test
    if( ++addr >= (int)(sizeof(gAddresses)/sizeof(gAddresses[0])) )
    {
        addr = 0;
    }
    
        ////////////////////////////////////////////////////////////////
        // Turn off output?
    if( gAddresses[addr].offMilli && gAddresses[addr].offMilli<millis() )
    {
            // Clear off time
        gAddresses[addr].offMilli = 0;
        
            // Disable output
        if( gAddresses[addr].isDigital )
        {
            digitalWrite( gAddresses[addr].outputPin, LOW);
        }else{
            analogWrite( gAddresses[addr].outputPin, 0);
        }
        
            // If still enabled and a flash type, set on time
        if( gAddresses[addr].output && gAddresses[addr].isFlasher)
        {
            gAddresses[addr].onMilli = millis() + gAddresses[addr].durationMilli;
        }else{
            gAddresses[addr].output = 0;
        }
        
        return;
    }
        
        ////////////////////////////////////////////////////////////////
        // Turn on output?
    if( gAddresses[addr].onMilli && gAddresses[addr].onMilli<=millis() )
    {
            // Clear off time
        gAddresses[addr].onMilli = 0;
        
            // Enable output
        if( gAddresses[addr].isDigital )
        {
            digitalWrite( gAddresses[addr].outputPin, HIGH);
        }else{
            analogWrite( gAddresses[addr].outputPin, gAddresses[addr].analogValue);
        }
        
            // If still enabled and a flash type, set off time
        if( gAddresses[addr].durationMilli )
        {
            gAddresses[addr].offMilli = millis() + gAddresses[addr].durationMilli;
        }
        
        return;
    }
  //******* DCC End *******************************************************************************

}


//******* NeoPixel Start *******************************************************************************
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();
       
        delay(wait);
       
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
//******* NeoPixel End *******************************************************************************
