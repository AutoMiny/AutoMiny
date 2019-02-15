// MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
// 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-11-25 - added delays to reduce sampling rate to ~200 Hz
//                   added temporizing printing during long computations
//      2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
//                   dynamic speed change when closing in
//      2016-10-22 - cosmetic changes
//      2016-10-19 - initial release of IMU_Zero
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release of MPU6050_RAW

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

  If an MPU6050
      * is an ideal member of its tribe,
      * is properly warmed up,
      * is at rest in a neutral position,
      * is in a location where the pull of gravity is exactly 1g, and
      * has been loaded with the best possible offsets,
then it will report 0 for all accelerations and displacements, except for
Z acceleration, for which it will report 16384 (that is, 2^14).  Your device
probably won't do quite this well, but good offsets will all get the baseline
outputs close to these target values.

  Put the MPU6050 in a flat and horizontal surface, and leave it operating for
5-10 minutes so its temperature gets stabilized.

  Run this program.  A "----- done -----" line will indicate that it has done its best.
With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take
a few minutes to get there.

  Along the way, it will generate a dozen or so lines of output, showing that for each
of the 6 desired offsets, it is
      * first, trying to find two estimates, one too low and one too high, and
      * then, closing in until the bracket can't be made smaller.

  The line just above the "done" line will look something like
    [567,567] --> [-1,2]  [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1]  [-25,-24] --> [0,3] [5,6] --> [0,4]
As will have been shown in interspersed header lines, the six groups making up this
line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
X gyro, Y gyro, and Z gyro, respectively.  In the sample shown just above, the trial showed
that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration,
and so on.

  The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN 6
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 21

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


#define BATTERY_PIN A6
#define ENABLE_PIN 7
const float referenceVolts = 5.2;
const float R1 = 3300.0;
const float R2 = 1490.0;

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;
uint16_t ledOffset = 0;
int16_t ledIncreaseVal = 1;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void displayVoltageGoodLed() {
    pixels.setPixelColor(0, 0, 255, 0);
    pixels.setPixelColor(10, 0, 255, 0);
    pixels.setPixelColor(11, 0, 255, 0);
    pixels.setPixelColor(20, 0, 255, 0);
    pixels.show();
}

void displayVoltageWarningLed() {
    pixels.setPixelColor(0, 255, 255, 0);
    pixels.setPixelColor(10, 255, 255, 0);
    pixels.setPixelColor(11, 255, 255, 0);
    pixels.setPixelColor(20, 255, 255, 0);
    pixels.show();
}

void displayVoltageBadLed() {
    pixels.setPixelColor(0, 255, 0, 0);
    pixels.setPixelColor(10, 255, 0, 0);
    pixels.setPixelColor(11, 255, 0, 0);
    pixels.setPixelColor(20, 255, 0, 0);
    pixels.show();
}

void displayWorkingLed(uint16_t offset) {
    pixels.clear();
    pixels.setBrightness(16);
    pixels.setPixelColor(0 + offset, 255, 255, 0);
    pixels.setPixelColor(10 - offset, 255, 255, 0);
    pixels.setPixelColor(11 + offset, 255, 255, 0);
    pixels.setPixelColor(20 - offset, 255, 255, 0);
    pixels.show();
}

void displayInitializeLed() {
    pixels.clear();
    pixels.setBrightness(16);
    for (uint8_t i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 0, 0);
    }
    pixels.show();
}

void displaySuccessLed() {
    pixels.clear();
    pixels.setBrightness(16);
    for (uint8_t i = 0; i < NUMPIXELS; i++) {
        pixels.setPixelColor(i, 255, 255, 0);
    }
    pixels.show();
}

void ForceHeader()
{ LinesOut = 99; }

void GetSmoothed()
{ int RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
    { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
    { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
            Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
            Sums[j] = Sums[j] + RawValue[j];
    } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
    { Smoothed[i] = (Sums[i] + N/2) / N ; }
} // GetSmoothed

void Initialize()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    pixels.begin();

    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
} // Initialize

void SetOffsets(int TheOffsets[6])
{ accelgyro.setXAccelOffset(TheOffsets [iAx]);
    accelgyro.setYAccelOffset(TheOffsets [iAy]);
    accelgyro.setZAccelOffset(TheOffsets [iAz]);
    accelgyro.setXGyroOffset (TheOffsets [iGx]);
    accelgyro.setYGyroOffset (TheOffsets [iGy]);
    accelgyro.setZGyroOffset (TheOffsets [iGz]);
} // SetOffsets

void ShowProgress()
{ if (LinesOut >= LinesBetweenHeaders)
    { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
    } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
    { Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
            Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
        { Serial.println(RBRACKET); }
        else
        { Serial.print("]\t"); }
    }
    LinesOut++;

    displayWorkingLed(ledOffset);

    if (ledOffset >= 5) {
        ledIncreaseVal = -1;
    }

    if (ledOffset <= 0) {
        ledIncreaseVal = 1;
    }

    ledOffset += ledIncreaseVal;
} // ShowProgress

void WriteEEPROMInt(int addr, int val) {
    byte low, high;
    low=val&0xFF;
    high=(val>>8)&0xFF;
    EEPROM.write(addr, low);
    EEPROM.write(addr+1, high);
    return;
}

void WriteEEPROM() {
    int addr = 0;
    // XAccel
    WriteEEPROMInt(addr, LowOffset[0]);
    addr += 2;
    // YAccel
    WriteEEPROMInt(addr, LowOffset[1]);
    addr += 2;
    // ZAccel
    WriteEEPROMInt(addr, LowOffset[2]);
    addr += 2;
    // XGyro
    WriteEEPROMInt(addr, LowOffset[3]);
    addr += 2;
    // YGyro
    WriteEEPROMInt(addr, LowOffset[4]);
    addr += 2;
    // ZGyro
    WriteEEPROMInt(addr, LowOffset[5]);
}

void SetAveraging(int NewN)
{ N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
} // SetAveraging

void PullBracketsIn()
{ boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];

    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
    { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
        { SetAveraging(NSlow); }
        else
        { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
        { if (HighOffset[i] <= (LowOffset[i]+1))
            { NewOffset[i] = LowOffset[i]; }
            else
            { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                { AllBracketsNarrow = false; }
            } // binary search
        }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // closing in
            if (Smoothed[i] > Target[i])
            { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
            } // use lower half
            else
            { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
            } // use upper half
        } // closing in
        ShowProgress();
    } // still working

} // PullBracketsIn

void PullBracketsOut()
{ boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();

    while (!Done)
    { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
            { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
            }
            else
            { NextLowOffset[i] = LowOffset[i]; }
        } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
        { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
            { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
            }
            else
            { NextHighOffset[i] = HighOffset[i]; }
        } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
        { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
        }
    } // keep going
} // PullBracketsOut

void TurnOnCar() {
    /***Voltmeter**/
    int value = analogRead(BATTERY_PIN);
    int enable_pin_status = LOW;
    float vout = (value * referenceVolts) / 1024.0;
    float measuredVoltage = vout / (R2/(R1+R2));

    //Check voltage, if it is below 13V then the indicator start blinking before turning off
    if (measuredVoltage > 14.0){
        enable_pin_status = HIGH;
    }
    else{
        enable_pin_status = LOW;
    }

    if (measuredVoltage > 14.5) {
        displayVoltageGoodLed();
    } else if (measuredVoltage <= 14.5 && measuredVoltage >= 13.8)
    {
        displayVoltageWarningLed();
    } else if (measuredVoltage < 13.7) {
        displayVoltageBadLed();
    }

    digitalWrite(ENABLE_PIN, enable_pin_status);
}

void TurnOffCar() {
    delay(5000);
    digitalWrite(ENABLE_PIN, LOW);
}

void setup()
{
    TurnOnCar();
    Initialize();
    displayInitializeLed();
    for (int i = iAx; i <= iGz; i++)
    { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel
        HighOffset[i] = 0;
        LowOffset[i] = 0;
    } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);

    PullBracketsOut();
    PullBracketsIn();
    WriteEEPROM();
    displaySuccessLed();

    Serial.println("-------------- done --------------");
    TurnOffCar();
} // setup

void loop()
{
} // loop

