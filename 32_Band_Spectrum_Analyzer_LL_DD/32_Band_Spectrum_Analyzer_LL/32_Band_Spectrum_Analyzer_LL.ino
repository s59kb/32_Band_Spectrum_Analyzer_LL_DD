/*
Copyright Redesigned 2021 kranjc.boki@gmail.com
25.11.2021
Redesigned for dual board Arduino nano stereo mode and add band selector,
band selector:
0 - 10 kHz
0 - 20 kHz

--------------------------------------------------------------------------
Copyright (c) 2019 Shajeeb TM
https://github.com/shajeebtm/Arduino-audio-spectrum-visualizer-analyzer/
https://create.arduino.cc/projecthub/Shajeeb/32-band-audio-spectrum-visualizer-analyzer-902f51

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
////////////////////////////////////////////////////////////////////////////////////
#include <arduinoFFT.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <EEPROM.h>
//-------------------------------------------------------------------------
#define SAMPLES 64    //Must be a power of 64
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW   // Set display type  so that  MD_MAX72xx library treets it properly
#define MAX_DEVICES  4   // Total number display modules 4
#define CLK_PIN   13  // Clock pin to communicate with display
#define DATA_PIN  11  // Data pin to communicate with display
#define CS_PIN    10  // Control pin to communicate with display
#define xres 32       // Total number of  columns in the display, must be <= SAMPLES/2  32
#define yres 8        // Total number of  rows in the display 8
//-------------------------------------------------------------------------
//int MY_ARRAY[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // default = standard pattern
int MY_ARRAY[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // BOKI pattern
int MY_MODE_1[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // standard pattern
int MY_MODE_2[]={0, 128, 64, 32, 16, 8, 4, 2, 1}; // only peak pattern
int MY_MODE_3[]={0, 128, 192, 160, 144, 136, 132, 130, 129}; // only peak +  bottom point
int MY_MODE_4[]={0, 128, 192, 160, 208, 232, 244, 250, 253}; // one gap in the top , 3rd light onwards
//int MY_MODE_5[]={0, 1, 3, 7, 15, 31, 63, 127, 255}; // standard pattern, mirrored vertically
int MY_MODE_5[]={0, 8, 28, 62, 127, 0, 0, 0, 0}; // BOKI_OSCI
//-------------------------------------------------------------------------
double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[xres];
//-------------------------------------------------------------------------
int yvalue;
int displaycolumn , displayvalue;
int peaks[xres];
int displaymode = 1;      // 1
int displayband = 1;      // 1
int OldMode = 0;          // 0
int OldBand = 0;          // 0
/*
// DESNI KANAL
int buttonBand = 2; // the number of the pushbutton pin 2
int buttonMode = 3;// the number of the pushbutton pin 3
int LedBand_0 = 4; // pin 4
int LedMode_1 = 5; // pin 5
int LedMode_2 = 6; // pin 6
int LedMode_3 = 7; // pin 7
int LedMode_4 = 8; // pin 8
int LedMode_5 = 9; // pin 9
*/
// LEVI KANAL
int InputBand_0 = 4; // pin 4
int InputMode_1 = 5; // pin 5
int InputMode_2 = 6; // pin 6
int InputMode_3 = 7; // pin 7
int InputMode_4 = 8; // pin 8
int InputMode_5 = 9; // pin 9

///////////////////////////////////////////////////////////////////////////////////////
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   // display object
arduinoFFT FFT = arduinoFFT();                                    // FFT object
///////////////////////////////////////////////////////////////////////////////////////
void setup() {
    ADCSRA = 0b11100101;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
    ADMUX = 0b00000000;       // use pin A0 and external voltage reference 
/*
    // DESNI KANAL  
    pinMode(buttonBand, INPUT); // sets the digital pin 2 as input
    pinMode(buttonMode, INPUT); // sets the digital pin 3 as input
    pinMode(LedBand_0, OUTPUT);  // sets the digital pin 4 as output      
    pinMode(LedMode_1, OUTPUT);  // sets the digital pin 5 as output
    pinMode(LedMode_2, OUTPUT);  // sets the digital pin 6 as output
    pinMode(LedMode_3, OUTPUT);  // sets the digital pin 7 as output
    pinMode(LedMode_4, OUTPUT);  // sets the digital pin 8 as output
    pinMode(LedMode_5, OUTPUT);  // sets the digital pin 9 as output      
*/   
    // LEVI KANAL  
    pinMode(InputBand_0, INPUT);  // sets the digital pin 4 as input      
    pinMode(InputMode_1, INPUT);  // sets the digital pin 5 as input
    pinMode(InputMode_2, INPUT);  // sets the digital pin 6 as input
    pinMode(InputMode_3, INPUT);  // sets the digital pin 7 as input
    pinMode(InputMode_4, INPUT);  // sets the digital pin 8 as input
    pinMode(InputMode_5, INPUT);  // sets the digital pin 9 as input    
    
    mx.begin();           // initialize display
    //mx.control(MD_MAX72XX::INTENSITY, 0); // 0-10
    delay(50);            // wait to get reference voltage stabilized
    displaymode = EEPROM.read(1);  //read EEPROM data at address 1
    displayband = EEPROM.read(2);  //read EEPROM data at address 2
}
///////////////////////////////////////////////////////////////////////////////////////
void loop() {
   // ++ Sampling
   for(int i=0; i<SAMPLES; i++) {
      while(!(ADCSRA & 0x10));    // wait for ADC to complete current conversion ie ADIF bit set
      //ADCSRA = 0b11110100 ;     // KRATKI
      //ADCSRA = 0b11110101 ;     // SREDNJI Clear ADIF bit so that ADC can do next operation (0xf5) 0b11110101 orginal
      //ADCSRA = 0b11110110 ;     // DUGI 
      //ADCSRA = 0b11110111 ;     // PREDUGI
      //-------------------------------------------------------------
      /*
      // DESNI KANAL     
      int ReadingBand = digitalRead(buttonBand); 
      if ((ReadingBand == 1)&&(OldBand == 0)) { // works only when pressed 
          displayband = displayband + 1;
          EEPROM.write(2, displayband);         //write value to current address counter address
          delay(100);
      }  
      OldBand = ReadingBand;
      if (displayband > 2) { // works only when pressed
          displayband = 1;
          EEPROM.write(2, displayband);         //write value to current address counter address
          delay(100);}      
      if (displayband == 1) {
        ADCSRA = 0b11110110 ;  // DUGI
        digitalWrite(LedBand_0, 1);}
      if (displayband == 2) {  
        ADCSRA = 0b11110101 ;  // SREDNJI Cclear ADIF bit so that ADC can do next operation (0xf5) 0b11110101 orginal
        digitalWrite(LedBand_0, 0);}
      */
      //-------------------------------------------------------------
      
      // LEVI KANAL
      if (digitalRead(InputBand_0)==1) {
        ADCSRA = 0b11110110 ;}  // DUGI
      else {  
        ADCSRA = 0b11110101 ;}  // SREDNJI Cclear ADIF bit so that ADC can do next operation (0xf5) 0b11110101 orginal
      
      //-------------------------------------------------------------
      int value = ADC - 512 ;                 // Read from ADC and subtract DC offset caused value
      vReal[i]= value/8;                      // Copy to bins after compressing
      vImag[i] = 0;                          
    }
    //-------------------------------------------------------------
    // -- Sampling
    // ++ FFT
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    // -- FFT
    //-------------------------------------------------------------
    // ++ re-arrange FFT result to match with no. of columns on display ( xres )
    int step = (SAMPLES/2)/xres; 
    int c=0;
    for(int i=0; i<(SAMPLES/2); i+=step)  
    {
      data_avgs[c] = 0;//0
      for (int k=0 ; k< step ; k++) {
          data_avgs[c] = data_avgs[c] + vReal[i+k];
      }
      data_avgs[c] = data_avgs[c]/step; 
      c++;
    }
    // -- re-arrange FFT result to match with no. of columns on display ( xres )
    //-------------------------------------------------------------
    // ++ send to display according measured value 
    for(int i=0; i<xres; i++)
    {
      data_avgs[i] = constrain(data_avgs[i],0,80);            // set max & min values for buckets
      data_avgs[i] = map(data_avgs[i], 0, 80, 0, yres);        // remap averaged values to yres
      yvalue=data_avgs[i];
      peaks[i] = peaks[i]-1;    // decay by one light
      if (yvalue > peaks[i]) 
          peaks[i] = yvalue ;
      yvalue = peaks[i];    
      displayvalue=MY_ARRAY[yvalue];
      // DESNI KANAL
      //displaycolumn=31-i;   // NISKI PREMA VISOKIM DESNA STRANA 31
      // LEVI KANAL      
      displaycolumn=i;   // VISOKI PREMA NISKIM LEVA STRANA
      mx.setColumn(displaycolumn, displayvalue);              // for left to right
     }
    // -- send to display according measured value 
    displayModeChange ();         // check if button pressed to change display mode
} 
///////////////////////////////////////////////////////////////////////////////////////
void displayModeChange() {

  // LEVI KANAL
  if (digitalRead(InputMode_1)==1) {
    for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_1[i];}
  } 
  if (digitalRead(InputMode_2)==1) {
    for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_2[i];}
  }   
  if (digitalRead(InputMode_3)==1) {
    for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_3[i];}
  } 
  if (digitalRead(InputMode_4)==1) {
    for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_4[i];}
  } 
  if (digitalRead(InputMode_5)==1) {
    for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_5[i];}
  }
 
//--------------------------------------------------------
/*
  // DESNI KANAL  
    int NewMode = digitalRead(buttonMode);
    if ((NewMode == 1) && (OldMode == 0)) { 
      displaymode = displaymode + 1;
      EEPROM.write(1, displaymode);         //write value to current address counter address
      digitalWrite(LedMode_1, 0);  // sets the LED to the MY_MODE_1
      digitalWrite(LedMode_2, 0);  // sets the LED to the MY_MODE_2
      digitalWrite(LedMode_3, 0);  // sets the LED to the MY_MODE_3
      digitalWrite(LedMode_4, 0);  // sets the LED to the MY_MODE_4
      digitalWrite(LedMode_5, 0);  // sets the LED to the MY_MODE_5
      delay(200);
    } 
    OldMode = NewMode;
    if (displaymode > 5) { 
      displaymode = 1;
      EEPROM.write(1, displaymode);         //write value to current address counter address 
    }
    if (displaymode == 1) {   //      
      digitalWrite(LedMode_2, 1);  // sets the LED to the MY_MODE_2
      for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_2[i];}
    }
    if (displaymode == 2) {   // 
      digitalWrite(LedMode_3, 1);  // sets the LED to the MY_MODE_3
      for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_3[i];}
    }
    if (displaymode == 3) {   // 
      digitalWrite(LedMode_4, 1);  // sets the LED to the MY_MODE_4
      for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_4[i];}
    }
    if (displaymode == 4) {   // 
      digitalWrite(LedMode_5, 1);  // sets the LED to the MY_MODE_5
      for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_5[i];}
    }
    if (displaymode == 5) {   //
      digitalWrite(LedMode_1, 1);  // sets the LED to the MY_MODE_1          
      for (int i=0 ; i<=8 ; i++ ) {
        MY_ARRAY[i]=MY_MODE_1[i];}
    }
*/  
}
///////////////////////////////////////////////////////////////////////////////////////
