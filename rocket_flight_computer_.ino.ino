// first define the mpu6050 x and y orientation 

#include<Wire.h>
#include<MPU6050.h>
#include<SD.h>
#include<SPI.h>
#include<SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include<FastLED.h>


// define the mpu6050 
MPU6050 mpu;
SoftwareSerial xbeeSerial(0, 1); // Connect XBee TX (DI) to Arduino D2, XBee RX (DO) to Arduino D3


Adafruit_BMP280 bmp; // Create a BMP280 instance

// define the buzzer value 
int buzzerpin = 9;
// define the pyro channel 
int pyro_ch = 14 ;

// define the flag to track stability 
bool isstable = true ;


const int delatmpu6050_on = 0;
const int deltampu6050_off = 0;

// How many leds in your strip?
#define NUM_LEDS 1


// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 2
#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];

const int deltax = 0;
const int deltay = 0;




void setup() {
    //   define the buzzer value 
  pinMode(buzzerpin , OUTPUT);
  digitalWrite(buzzerpin , LOW);
//   define the pyro channel 
  pinMode(pyro_ch , OUTPUT);
  digitalWrite(pyro_ch , LOW );

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); //define for fast led 

  Wire.begin();

  if (delatmpu6050_on  == deltampu6050_off)
  {
     mpu.initialize();
     digitalWrite(buzzerpin , HIGH);
     delay(100);
     digitalWrite(buzzerpin, LOW );
     delay(100);
     digitalWrite(buzzerpin , HIGH);
     delay(100);
     digitalWrite(buzzerpin, LOW );
     delay(100);
     digitalWrite(buzzerpin , HIGH);
     delay(100);
     digitalWrite(buzzerpin, LOW );
     delay(100);
     digitalWrite(buzzerpin , HIGH);
     delay(100);
     digitalWrite(buzzerpin, LOW );
     delay(100);

     for (int  i = 0; i <= 5; i++)
     {
        mpu6050_color();
        /* code */
     }
     
 
     
 /* code */
  }
  
 
  bmp.begin(9600);
  bmp.begin(0x76);
  Serial.begin(9600);
  xbeeSerial.begin(9600);
  
  if (!SD.begin(10)) {
    Serial.println("SD Card initialization failed.");
    //   again define the led 
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(10000);
    //  now turn off the led 
    leds[0]= CRGB::Black;
    FastLED.show();
    delay(10000);
    
    while (1);
  }
  else{
    Serial.println("sd card initilization done ");
    Serial.println("done");
    if (deltax == deltay)
    {
        sd_card_light();
        /* code */
    }
    
    //   again define the led 

  }

  // Create a new file on the SD card for data logging
  File dataFile = SD.open("text.txt", FILE_WRITE);
  dataFile.println("agni_rocket Data:");
  dataFile.close();

alldonebeep();


}





void loop() {

    // define the bmp280 sensor
    float pressure = bmp.readPressure();

    // Calculate altitude based on standard atmospheric pressure
  float seaLevelPressure = 997.2; // Standard sea level pressure in hPa
  float altitude_m = bmp.readAltitude(seaLevelPressure);


// / define the roll and pitch 
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate pitch and roll angles
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll = atan2(-ay, az) * 180.0 / PI;
 

  // Calculate vertical velocity based on acceleration (m/s^2)
  float verticalVelocity = (float)ay * 9.81 / 1000; // Convert from sensor units to m/s^2
 
//  16384.0

// DEFINE FOR THE PYRO CHANNEL 
  mpu.getAcceleration(&ax ,&ay , &az);


//   calculate the angle in x-pitch 
 float pitch2 = atan2 (-ax,sqrt(ay*ay+az*az)) * RAD_TO_DEG;



    //  // Check if the sensor is stable (within a threshold)
  if (abs(pitch2) <= 60.0) {
    isstable = true;
    digitalWrite(buzzerpin, LOW); // Turn off the buzzer
    // define the pyro channel 
    digitalWrite(pyro_ch , LOW );
  } else {
    if (isstable) {
      // If the sensor tilts after being stable, trigger the buzzer
      digitalWrite(buzzerpin, HIGH); // Turn on the buzzer
      digitalWrite(pyro_ch , HIGH);
          //   again define the led 
      leds[0] = CRGB::Orange;
      FastLED.show();

//  now turn off the led 
      delay(5000); // delay for 5 second 
      digitalWrite(buzzerpin, LOW); // Turn off the buzzer
      digitalWrite(pyro_ch , LOW );
       leds[0]= CRGB::Black;
       FastLED.show();
    //    digitalWrite(pyro_ch , LOW );
    // //   define the pyro channel 
    //   digitalWrite(pyro_ch , HIGH);
    //   delay(10000);
    //   digitalWrite(pyro_ch , LOW );
      isstable = false;
    }
  }




    // Print altitude in meters in serial monitor 

  Serial.print("Altitude: ");
  Serial.print(altitude_m);
  Serial.println(" meters");
  Serial.print("pitch");
  Serial.println(pitch);
  Serial.print("angle");
  Serial.print ("roll");
  Serial.println(roll);
  Serial.print ("angle");
  
  // Save data to SD card
  File dataFile = SD.open("text.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis() / 1000.0); // Time in seconds
    dataFile.print(",");
    dataFile.print("altitude=BMP280");dataFile.print(altitude_m);
    dataFile.print("verticalvelocity = MPU6050");dataFile.print(verticalVelocity);
    dataFile.print("pitch") ;dataFile.print(pitch);
    dataFile.print("roll");dataFile.print(roll);
    dataFile.println(";");

    dataFile.close();
  } else {
    Serial.println("Error opening data file.");
  }


//   // Create a data string
//   String data = String(pitch, 4) + "," + String(roll, 4)+","+String(verticalVelocity) +"," +String(altitude_m);
  
//   // Send data via XBee
//   xbeeSerial.println(data);
// //   xbeeSerial.print(millis() / 1000.0 ); // Time in seconds
//     // Send data via XBee


  // Send data via XBee

//   define the bmp280 sendor in xbee pro s2c 
  xbeeSerial.print(millis() / 1000.0); // Time in seconds
  xbeeSerial.print(",");
  xbeeSerial.println(altitude_m);

  // define the led 
ledforblick();


  
  // Delay before next reading
  delay(100);
}

// define the function for sd card 
void sd_card_light(){
    //   again define the led 
 leds[0] = CRGB::Orange;
 FastLED.show();
 delay(500);
//  now turn off the led 
 leds[0]= CRGB::Black;
 FastLED.show();
 delay(100);

  leds[0] = CRGB::Blue;
 FastLED.show();
 delay(500);
//  now turn off the led 
 leds[0]= CRGB::Black;
 FastLED.show();
 delay(100);
  leds[0] = CRGB::Green;
 FastLED.show();
 delay(500);
//  now turn off the led 
 leds[0]= CRGB::Black;
 FastLED.show();
 delay(100);


}

void mpu6050_color(){
     leds[0] = CRGB::Orange;
 FastLED.show();
 delay(1000);
// //  now turn off the led 
//  leds[0]= CRGB::Black;
//  FastLED.show();
//  delay(100);

  leds[0] = CRGB::Green;
 FastLED.show();
 delay(1000);
// //  now turn off the led 
//  leds[0]= CRGB::Black;
//  FastLED.show();
//  delay(100);
  leds[0] = CRGB::White;
 FastLED.show();
 delay(1000);
// //  now turn off the led 
//  leds[0]= CRGB::Black;
//  FastLED.show();
//  delay(100);

}

void alldonebeep(){

digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(100);
digitalWrite(buzzerpin , LOW );
delay(100);
digitalWrite(buzzerpin , HIGH);
delay(1000);
digitalWrite(buzzerpin , LOW );
delay(1000);
}

void ledforblick(){
 leds[0] = CRGB::Green;
 FastLED.show();
 delay(10);
//  now turn off the led 
 leds[0]= CRGB::Black;
 FastLED.show();
 delay(10);
}
