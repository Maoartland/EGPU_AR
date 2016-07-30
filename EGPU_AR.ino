#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;       // hard-wired for UNO shields anyway.
#include <TouchScreen.h>
#include <MsTimer2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>

#include<CountUpDownTimer.h>
CountUpDownTimer T(UP, HIGH); // Default precision is HIGH, but you can change it to also be LOW
unsigned long Watch, _micro, time = micros();
unsigned int Clock = 0, R_clock;
boolean Reset = false, Stop = false, Paused = false, _type;
volatile boolean timeFlag = false;
#define DOWN 0
#define UP 1


#define ONE_WIRE_BUS 13
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

// most mcufriend shields use these pins and Portrait mode:
uint8_t YP = A1;  // must be an analog pin, use "An" notation!
uint8_t XM = A2;  // must be an analog pin, use "An" notation!
uint8_t YM = 7;   // can be a digital pin
uint8_t XP = 6;   // can be a digital pin
uint8_t SwapXY = 0;

uint16_t TS_LEFT = 920;
uint16_t TS_RT  = 150;
uint16_t TS_TOP = 940;
uint16_t TS_BOT = 120;
char *name = "Unknown controller";

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;

#define MINPRESSURE 20
#define MAXPRESSURE 1000

#define SWAP(a, b) {uint16_t tmp = a; a = b; b = tmp;}

int16_t BOXSIZE;
int16_t PENRADIUS = 3;
uint16_t identifier, oldcolor, currentcolor;
uint8_t Orientation = 0;    //PORTRAIT

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

//Bitmap SpalshScreen
static const unsigned char PROGMEM  MMLogo [] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
  0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};

void show_logo (){
  tft.drawBitmap(0 ,0 , MMLogo, 240 , 48, 0xFFFF);
}

float temps[3];

volatile bool tempsUpdated = false;
int Watts_fake = 0;

void timerEvent() {

  for (int i = 0; i < 3; i++)
    temps[i] = sensors.getTempCByIndex(i);

    Watts_fake = temps[0] * 11;

  tempsUpdated = true;

  sensors.requestTemperatures();
}

/*void Watts (){
  Watts_fake = temps[0] * 12;
}*/

float lastTemps[3];
float lastClock[3];
int lastHours=0;
int lastMinutes=0;
int lastSeconds=0;
int lastWatts_fake=0;

void show_HomeScreen (){
  int16_t x, y;

  //tft.setTextSize(2);
  //tft.fillRect(0, 0, 240, 8, RED); //Bar 1
  tft.setTextColor(WHITE);
  tft.setFont(&FreeSansBold12pt7b);

  tft.setCursor(72, 26);

  tft.println("Gtx 980ti");
  tft.fillRect(40, 40, 160, 4, RED);

  tft.setTextSize(2);
  //tft.setTextWrap(false);
  tft.setCursor(55, 96);
  
  if (lastTemps[0] != temps[0]){
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[0], 1);
  tft.setCursor(x, y);

  if ((temps[0]) < 23) {tft.setTextColor(WHITE);}
  else if ((temps[0]) >= 23 && (temps[0] <= 25)) {tft.setTextColor(YELLOW);}
  else if ((temps[0]) > 25) {tft.setTextColor(RED);}

  tft.print(temps[0], 1);

  tft.setCursor(140,96);
  tft.println(" C\n");}

  //tft.print("Power: ");
  //if ((W) < 100) {tft.setTextColor(WHITE);}
  //else if ((W) >= 100 && (temps[0] <= 200)) {tft.setTextColor(YELLOW);}
  //else if ((W) > 200) {tft.setTextColor(RED);}

  tft.setCursor(55,148);
  if (lastTemps[1] != temps[1]){
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastWatts_fake);
  tft.setCursor(x, y);
  
  if ((Watts_fake) < 100) {tft.setTextColor(WHITE);}
  else if ((Watts_fake) >= 150 && (Watts_fake <= 249)) {tft.setTextColor(YELLOW);}
  else if (Watts_fake > 249) {tft.setTextColor(RED);}

  tft.print(Watts_fake);
  lastWatts_fake = Watts_fake;
  
  tft.setCursor(145,148);
  tft.println("W");}

  tft.fillRect(40, 164, 160, 4, YELLOW);

  tft.setCursor(52 , 200);
  tft.setTextSize(1);

  if (lastTemps[1] != temps[1]){
  tft.setTextColor(WHITE);
  tft.print("Egpu : ");
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[1], 1);
  tft.setCursor(x, y);

  if ((temps[1]) < 23) {tft.setTextColor(WHITE);}
  else if ((temps[1]) >= 23 && (temps[1] <= 25)) {tft.setTextColor(YELLOW);}
  else if ((temps[1]) > 25) {tft.setTextColor(RED);}

  tft.print(temps[1], 1);
  tft.print(" c\n");}

  tft.setCursor(55 , 230);

  if (lastTemps[2] != temps[2]){
  tft.setTextColor(WHITE);
  tft.print("Psu   : ");
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[2], 1);
  tft.setCursor(x, y);

  if ((temps[2]) < 23) {tft.setTextColor(WHITE);}
  else if ((temps[2]) >= 23 && (temps[2] <= 25)) {tft.setTextColor(YELLOW);}
  else if ((temps[2]) > 25) {tft.setTextColor(RED);}

  //tft.setTextColor(WHITE);
  tft.print(temps[2], 1);
  tft.print(" c\n");}

  tft.fillRect(40, 248, 160, 4, GREEN);

  for (int i = 0; i < 3; i++)
    lastTemps[i] = temps[i];
    

    T.Timer(); // run the timer
    
    //Hours
    tft.setTextSize(2);
    tft.setCursor(33,300);
    
    if (lastHours != T.ShowHours());{
      x = tft.getCursorX();
      y = tft.getCursorY();
      tft.setTextColor(BLACK);

      char lastH[10];
      if(T.ShowHours() != 0 && T.ShowHours() < 10 && T.ShowHours() != 10 && T.ShowHours() != 59){
        sprintf(lastH, "0%d", lastHours);
        tft.print(lastH);
        }else if (T.ShowHours() == 10){
         tft.print("09");
        }else if (T.ShowHours() == 0 && lastHours != 0){
         tft.print("59");
        }else{
         tft.print(lastHours);
        }

      tft.setCursor(x, y);
      tft.setTextColor(WHITE);
  
      if(T.ShowHours() >= 0 && T.ShowHours() < 10){
        tft.print("0");tft.print(T.ShowHours());
        }else{
          tft.print(T.ShowHours());
        }
      }
    
  
    tft.setCursor(87,300);
    tft.print(":");

    //Minutes

    tft.setCursor(99,300);

    if (lastMinutes != T.ShowMinutes());{
      x = tft.getCursorX();
      y = tft.getCursorY();
      tft.setTextColor(BLACK);

      char lastMin[10];
      if(T.ShowMinutes() != 0 && T.ShowMinutes() < 10 && T.ShowMinutes() != 10 && T.ShowMinutes() != 59){
        sprintf(lastMin, "0%d", lastMinutes);
        tft.print(lastMin);
        }else if ((T.ShowMinutes()) == 10){
          tft.print("09");
        }else if ((T.ShowMinutes()) == 0 && lastMinutes != 0){
          tft.print("59");
        }else{
          tft.print(lastMinutes);
        }
        
      tft.setCursor(x, y);
      tft.setTextColor(WHITE);

      char Min[10];
      if(T.ShowMinutes() >= 0 && T.ShowMinutes() < 10){
        sprintf(Min, "0%d", T.ShowMinutes());
        tft.print(Min);
      }else{
        tft.print(T.ShowMinutes());
      }
     }
    
    tft.setCursor(153,300);
    tft.print(":");
    

    //Seconds

    char lastSec[10];
    if (lastSeconds != T.ShowSeconds());{
    x = tft.getCursorX();
    y = tft.getCursorY();
    tft.setTextColor(BLACK);


    if(T.ShowSeconds() != 0 && T.ShowSeconds() < 10 && T.ShowSeconds() != 10 && T.ShowSeconds() != 59){
      sprintf(lastSec, "0%d", lastSeconds);
      tft.print(lastSec);
    }else if (T.ShowSeconds() == 10){tft.print("09");
    }else if (T.ShowSeconds() == 0){tft.print("59");
    }else{
     tft.print(lastSeconds);
    }
      
    tft.setCursor(x, y);
    tft.setTextColor(WHITE);

    char sec[10];
  
    if(T.ShowSeconds() >= 0 && T.ShowSeconds() < 10){
      sprintf(sec, "0%d", T.ShowSeconds());
      tft.print(sec);
    }else{
      tft.print(T.ShowSeconds());
    }
   }

    tft.setTextSize(1);
    lastHours = T.ShowHours();
    lastMinutes = T.ShowMinutes();
    lastSeconds = T.ShowSeconds();
    
    // This DOES NOT format the time to 0:0x when seconds is less than 10.
    // if you need to format the time to standard format, use the sprintf() function.

}


void show_PowerSceen (){
  int16_t x, y;


  //tft.setTextSize(2);
  tft.setTextColor(YELLOW);
  tft.setFont(&FreeSansBold12pt7b);

  tft.setCursor(0, 20);

  tft.println("GTx 980Ti :");
  //tft.fillRect(0, 30, 240, 8, RED);
  tft.setTextSize(2);
  //tft.setTextWrap(false);
  tft.setCursor(40, 80);
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[0], 1);
  tft.setCursor(x, y);
  tft.setTextColor(WHITE);
  tft.print(temps[0], 1);
  tft.println("c\n");

  //tft.print("Power: ");
  tft.setCursor(55,140);
  tft.print("300");
  tft.println(" w");

  tft.setTextSize(1);
  tft.setCursor(45 , 200);
  tft.print("Psu  : ");
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[2], 1);
  tft.setCursor(x, y);
  tft.setTextColor(WHITE);
  tft.print(temps[2], 1);
  tft.print("c\n");

  tft.setCursor(40 , 230);
  tft.print("Egpu : ");
  x = tft.getCursorX();
  y = tft.getCursorY();
  tft.setTextColor(BLACK);
  tft.print(lastTemps[1], 1);
  tft.setCursor(x, y);
  tft.setTextColor(WHITE);
  tft.print(temps[1], 1);
  tft.print("c\n");





  tft.setTextSize(2);
  tft.setCursor(30,300);
  tft.print("00:00:00");
  tft.setTextSize(1);

  lastWatts_fake = Watts_fake;

  for (int i = 0; i < 3; i++)
    lastTemps[i] = temps[i];
}

void setup(void){

  Serial.begin(9600);
  Serial.println("MagicMods Egpu");
  Serial.println("Arduino Digital Temperature // Serial Monitor Version"); //Print a message
  sensors.begin();
  delay(100);

  //T.SetTimer(0,0,20);     //start at 1 minute (USE FOR: DOWN ONLY)
  //T.SetStopTime(0,0,30); // stop at 10 seconds (USE FOR: UP/DOWN)
  T.StartTimer();


  //delay(3000); //Can be used for boot relay delay windows (take in account boot/load time (how to calcul?))

  //attachInterrupt(digitalPinToInterrupt(pushButton), buttonPressed, RISING);

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  MsTimer2::set(1000, timerEvent);
  MsTimer2::start();


    uint16_t tmp;
    tft.begin(9600);

    tft.reset();
    identifier = 0x7789;
        YP = A2; XM = A1; YM = 7; XP = 6;
        TS_LEFT = 906; TS_RT = 169; TS_TOP = 161; TS_BOT = 919;


    ts = TouchScreen(XP, YP, XM, YM, 300);     //call the constructor AGAIN with new values.
    tft.begin(identifier);

    tft.fillScreen(BLACK);
    show_logo();
    //delay(2000);
    tft.fillScreen(BLACK);
    show_HomeScreen();
}

bool drawing = false;
uint16_t lastx, lasty;
int touchC = 0;
int lasttouchC = 0;
bool pageChanged = false;

void touchEvent(uint16_t xpos, uint16_t ypos)
{
    if (!drawing)
    {
      touchC ++;
      if (touchC == 3)
        touchC = 0;
      pageChanged = true;
      //Serial.println(touchC);
    }
    drawing = true;
}

void releaseEvent()
{
    drawing = false;
}

void pagetodisplay()
{
    if(touchC == 0){show_HomeScreen();}
    else if(touchC == 1){show_PowerSceen();}
    else if(touchC == 2){show_logo();}
}

void loop()
{
    uint16_t xpos, ypos;  //screen coordinates
    tp = ts.getPoint();   //tp.x, tp.y are ADC values

    // if sharing pins, you'll need to fix the directions of the touchscreen pins
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    pinMode(XP, OUTPUT);
    pinMode(YM, OUTPUT);
    //    digitalWrite(XM, HIGH);
    //    digitalWrite(YP, HIGH);
    // we have some minimum pressure we consider 'valid'
    // pressure of 0 means no pressing!

    //Serial.println(String(tp.x) + " " + String(tp.y) + " " + String(tp.z));

    if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) {
        // is controller wired for Landscape ? or are we oriented in Landscape?
        if (SwapXY != (Orientation & 1)) SWAP(tp.x, tp.y);
        // scale from 0->1023 to tft.width  i.e. left = 0, rt = width
        // most mcufriend have touch (with icons) that extends below the TFT
        // screens without icons need to reserve a space for "erase"
        // scale the ADC values from ts.getPoint() to screen values e.g. 0-239
        xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
        ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());

        touchEvent(xpos, ypos);
    }
    else
    {
        releaseEvent();
    }

  if (pageChanged) {
    tft.fillScreen(BLACK);
  }

  if (pageChanged || tempsUpdated) {
    pagetodisplay();
  }

  pageChanged = false;
  tempsUpdated = false;
}
