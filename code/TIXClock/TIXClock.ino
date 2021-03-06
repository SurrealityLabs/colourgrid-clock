#include <Wire.h>
#include "TimerOne.h"
#include "RTC.h"
#include "DateTime.h"
#include <util/delay.h>
#ifndef cbi
#define cbi(register,bit)       register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)       register |= (_BV(bit))
#endif

#include "CommandShell.h"
CommandShell CommandLine;

RTC_DS1307 RTC;

volatile uint8_t displayArray[3][32];
uint8_t newDisplayArray[3][32];
volatile uint8_t currentRow;
volatile uint8_t NumBlanks = 0;

uint8_t minuteOnesColour[] = {
  255, 0, 0};
uint8_t minuteTensColour[] = {
  0, 255, 0};
uint8_t hourOnesColour[] = {
  0, 0, 255};
uint8_t hourTensColour[] = {
  127, 0, 127};

#define MaxBlanks 2

// BLANK on OC2B / PD3
// GSCLK on OC0B / PD5
// MODE on OC1A / PB1
// XLAT on SS / OC1B / PB2
// SCLK on SCK / PB5
// DAT on MOSI / PB3

// ROW0 on PC0
// ROW1 on PC1
// ROW2 on PC2

/* UART command set array, customized commands may add here */
commandshell_cmd_struct_t uart_cmd_set[] =
{
  {
    "setDate", "\tsetDate [day] [month] [year]", setDateFunc      }
  ,
  {
    "setTime", "\tsetTime [hours] [minutes] [seconds]", setTimeFunc      }
  ,
  {
    "printTime", "\tprintTime", printTimeFunc      }
  ,
  {
    0,0,0      }
};

unsigned char daysInMonth[] = {
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

int setDateFunc(char * args[], char num_args) {
  if(3 != num_args) {
    Serial.println(F("Insufficient arguments!"));
    return 1;
  }

  int dayNum = atoi(args[0]);
  int monthNum = atoi(args[1]);
  int yearNum = atoi(args[2]);

  if((yearNum < 2000) || (yearNum > 2100)) {
    Serial.println(F("Invalid year!"));
    return 2;
  }

  if((monthNum < 1) || (monthNum > 12)) {
    Serial.println(F("Invalid month!"));
    return 3;
  }

  // Leap year handling
  if(monthNum == 2) {
    if(yearNum % 4 == 0) {
      if(yearNum % 100 == 0) {
        if(yearNum % 400 == 0) {
          daysInMonth[1] = 29;//is a leap year
        } 
        else {
          daysInMonth[1] = 28;//not a leap year
        } 
      } 
      else {
        daysInMonth[1] = 29;//is a leap year
      }
    } 
    else {
      daysInMonth[1] = 28;// not a leap year
    } 
  }

  if((dayNum < 1) || (dayNum > (daysInMonth[monthNum - 1]))) {

    Serial.println(F("Invalid day!"));
    return 4;
  }
  DateTime setTime = RTC.now();
  setTime.setYear(yearNum);
  setTime.setMonth(monthNum);
  setTime.setDay(dayNum);
  RTC.adjust(setTime);

  Serial.print(F("Setting date to "));
  Serial.print(dayNum);
  Serial.print('/');
  Serial.print(monthNum);
  Serial.print('/');
  Serial.println(yearNum);
  return 0;  
}

int setTimeFunc(char * args[], char num_args) {
  if(3 != num_args) {
    Serial.println(F("Insufficient arguments!"));
    return 1;
  }

  int hourNum = atoi(args[0]);
  int minNum = atoi(args[1]);
  int secNum = atoi(args[2]);

  if((hourNum < 0) || (hourNum > 23)) {
    Serial.println(F("Invalid hours!"));
    return 2;
  }

  if((minNum < 0) || (minNum > 59)) {
    Serial.println(F("Invalid minutes!"));
    return 3;
  }

  if((secNum < 0) || (secNum > 59)) {

    Serial.println(F("Invalid seconds!"));
    return 4;
  }
  DateTime setTime = RTC.now();
  setTime.setHour(hourNum);
  setTime.setMinute(minNum);
  setTime.setSecond(secNum);
  RTC.adjust(setTime);

  Serial.print(F("Setting time to "));
  Serial.print(hourNum);
  Serial.print(F(":"));
  Serial.print(minNum);
  Serial.print(F(":"));
  Serial.println(secNum);
  return 0;  
}

int printTimeFunc(char * args[], char num_args) {
  Serial.print(F("The current time is:"));
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  return 0;
}

void setup(void) {
  clearPixels();

  DDRB = 0xFF;
  DDRC = 0x07;
  DDRD = 0x3D;

  //we use OCR0A in to set frequency, OCR0B is Fast PWM mode to generate GSCLK
  TCCR0A = ((0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00));
  TCCR0B = ((0 << FOC0A) | (0 << FOC0B) | (1 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00));
  //no interrupts for timer0
  TIMSK0 = ((0 << OCIE0B) | (0 << OCIE0A) | (0 << TOIE0));
  //these values give us a grayscale clock of 4 megahertz with about a 40% duty cycle
  OCR0A = 2;
  OCR0B = 1;

  //using Timer1 to generate blanking signal. Set to source T1 pin, which is the same as OCR0B
  //using CTC mode with OCR1A and prescaler set so we get an interrupt every 4096 clocks
  TCCR1A = ((0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10));
  TCCR1B = ((0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS11) | (1 << CS10));
  TCCR1C = ((0 << FOC1A) | (0 << FOC1B));
  TIMSK1 = ((0 << ICIE1) | (0 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1));

  //this gives us a count of 4096 in OCR1A
  OCR1AH = 16;
  OCR1AL = 0;
  OCR1BH = 0;
  OCR1BL = 0;

  SPISetup();

  sei();


  Serial.begin(9600);
  Serial.println(F("Starting"));

  Wire.begin();
  RTC.begin();
  if (! RTC.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  DateTime now = RTC.now();
  randomSeed(now.unixtime());
  Serial.println(F("RTC setup"));
  CommandLine.commandTable = uart_cmd_set;
  CommandLine.init(&Serial);
}

ISR(TIMER1_COMPA_vect)
{
  //this is called every 4096 grayscale clocks
  //set blank high
  sbi(PORTD, 3);
  //if it's time to change colours, change the state of the state machine. we'll set blank low and reset the timer after we send new data.
    //otherwise, set blank low and reset Timer1 so we get a full 4096 clocks until the next blank
  if(currentRow == 0) {
    PORTC &= 0xF8;
    PORTC |= 0x06;
  } 
  else if(currentRow == 1) {
    PORTC &= 0xF8;
    PORTC |= 0x05;
  } 
  else if(currentRow == 2) {
    PORTC &= 0xF8;
    PORTC |= 0x03;
  }

  cbi(PORTB, 1);
  _delay_us(1);
  for(uint8_t i=0;i<32;i+=2) {
    SPIWriteByte(displayArray[currentRow][i]);
    SPIWriteByte(0x00 | (displayArray[currentRow][i + 1] >> 4));
    SPIWriteByte((displayArray[currentRow][i + 1] << 4) | 0x00);
  }
  sbi(PORTB, 2);
  _delay_us(10);
  cbi(PORTB, 2);
  _delay_us(1);
  cbi(PORTB, 1);
  _delay_us(1);

  cbi(PORTD, 3);
  TCNT1L = 0;
  TCNT1H = 0;
  currentRow++;
  if(currentRow >= 3) currentRow = 0;

}


void loop(void) {
  DateTime now = RTC.now();
  CommandLine.runService();

  static uint8_t alreadyRan = 2;

  uint8_t ShuffleData[9];
  uint8_t minuteOnes = now.minute() % 10;
  uint8_t minuteTens = now.minute() / 10;
  uint8_t hourOnes = now.hour() % 10;
  uint8_t hourTens = now.hour() / 10;

  uint8_t r, g, b, tempH, lastH;

  if(((now.second() % 30 == 0) && (alreadyRan == 1)) || (alreadyRan == 2)) {
    tempH = random(255);
    h2rgb(tempH, r, g, b);
    minuteOnesColour[0] = r;
    minuteOnesColour[1] = g;
    minuteOnesColour[2] = b;

    if(now.minute() > 9) {
      lastH = tempH;
      while(!checkDifference(lastH, tempH, 75)) tempH = random(255);
      h2rgb(tempH, r, g, b);
      minuteTensColour[0] = r;
      minuteTensColour[1] = g;
      minuteTensColour[2] = b;
    }

    if(now.hour() % 10 != 0) {
      lastH = tempH;
      while(!checkDifference(lastH, tempH, 75)) tempH = random(255);
      h2rgb(tempH, r, g, b);
      hourOnesColour[0] = r;
      hourOnesColour[1] = g;
      hourOnesColour[2] = b;
    }

    lastH = tempH;
    while(!checkDifference(lastH, tempH, 75)) tempH = random(255);
    h2rgb(tempH, r, g, b);
    hourTensColour[0] = r;
    hourTensColour[1] = g;
    hourTensColour[2] = b;

    clearPixels();

    // Hours 10 digit...
    for (uint8_t i = 0; i < 6; ++i)
      ShuffleData[i] = (i < hourTens) ? 1 : 0;

    ShuffleIfNeeded(ShuffleData, 3, hourTens);
    if(ShuffleData[0]) setPixel(8,0,hourTensColour[0],hourTensColour[1],hourTensColour[2]);
    if(ShuffleData[1]) setPixel(8,1,hourTensColour[0],hourTensColour[1],hourTensColour[2]);
    if(ShuffleData[2]) setPixel(8,2,hourTensColour[0],hourTensColour[1],hourTensColour[2]);

    // Hours 1 digit...
    for (uint8_t i = 0; i < 9; ++i)
      ShuffleData[i] = (i < hourOnes) ? 1 : 0;

    ShuffleIfNeeded(ShuffleData, 9, hourOnes);
    if(ShuffleData[0]) setPixel(7,0,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[1]) setPixel(7,1,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[2]) setPixel(7,2,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[3]) setPixel(6,0,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[4]) setPixel(6,1,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[5]) setPixel(6,2,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[6]) setPixel(5,0,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[7]) setPixel(5,1,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);
    if(ShuffleData[8]) setPixel(5,2,hourOnesColour[0],hourOnesColour[1],hourOnesColour[2]);

    // Minutes 10 digit...
    for (uint8_t i = 0; i < 9; ++i)
      ShuffleData[i] = (i < minuteTens) ? 1 : 0;

    ShuffleIfNeeded(ShuffleData, 6, minuteTens);
    if(ShuffleData[0]) setPixel(4,0,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);
    if(ShuffleData[1]) setPixel(4,1,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);
    if(ShuffleData[2]) setPixel(4,2,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);
    if(ShuffleData[3]) setPixel(3,0,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);
    if(ShuffleData[4]) setPixel(3,1,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);
    if(ShuffleData[5]) setPixel(3,2,minuteTensColour[0],minuteTensColour[1],minuteTensColour[2]);

    // Minutes 1 digit...
    for (uint8_t i = 0; i < 9; ++i)
      ShuffleData[i] = (i < minuteOnes) ? 1 : 0;

    ShuffleIfNeeded(ShuffleData, 9, minuteOnes);
    if(ShuffleData[0]) setPixel(2,0,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[1]) setPixel(2,1,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[2]) setPixel(2,2,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[3]) setPixel(1,0,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[4]) setPixel(1,1,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[5]) setPixel(1,2,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[6]) setPixel(0,0,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[7]) setPixel(0,1,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);
    if(ShuffleData[8]) setPixel(0,2,minuteOnesColour[0],minuteOnesColour[1],minuteOnesColour[2]);

    swapDisplay();
    alreadyRan = 0;
  } 
  else if(now.second() % 30 == 1) {
    alreadyRan = 1;
  }
}

void DoShuffle(uint8_t ShuffleData[9], uint8_t NumElems)
{
  for (uint8_t i = NumElems - 1; i > 0; --i)
  {   // Swap ShuffleData[i] with a random element from the set of 
    // ShuffleData[0..i] (yes, possibly itself.)
    uint8_t SwapElemIdx = random() % (i + 1);

    if (SwapElemIdx == i)
      continue; // Swapping the current element with itself.

    uint8_t Temp = ShuffleData[i];
    ShuffleData[i] = ShuffleData[SwapElemIdx];
    ShuffleData[SwapElemIdx] = Temp;
  }
}

inline void ShuffleIfNeeded(uint8_t ShuffleData[9], uint8_t NumElems, uint8_t NumSetElems)
{
  // Don't bother shuffling when all elements have an identical value.
  if (NumSetElems != NumElems && NumSetElems != 0)
    DoShuffle(ShuffleData, NumElems);
}

void h2rgb(uint8_t Hint, uint8_t& R, uint8_t& G, uint8_t& B) {

  int var_i;
  float H, S=1, V=1, var_1, var_2, var_3, var_h, var_r, var_g, var_b;

  H = (float)Hint / 256.0;

  if ( S == 0 )                       //HSV values = 0 ÷ 1
  {
    R = V * 255;
    G = V * 255;
    B = V * 255;
  }
  else
  {
    var_h = H * 6;
    if ( var_h == 6 ) var_h = 0;      //H must be < 1
    var_i = int( var_h ) ;            //Or ... var_i = floor( var_h )
    var_1 = V * ( 1 - S );
    var_2 = V * ( 1 - S * ( var_h - var_i ) );
    var_3 = V * ( 1 - S * ( 1 - ( var_h - var_i ) ) );

    if      ( var_i == 0 ) {
      var_r = V     ;
      var_g = var_3 ;
      var_b = var_1 ;
    }
    else if ( var_i == 1 ) {
      var_r = var_2 ;
      var_g = V     ;
      var_b = var_1 ;
    }
    else if ( var_i == 2 ) {
      var_r = var_1 ;
      var_g = V     ;
      var_b = var_3 ;
    }
    else if ( var_i == 3 ) {
      var_r = var_1 ;
      var_g = var_2 ;
      var_b = V     ;
    }
    else if ( var_i == 4 ) {
      var_r = var_3 ;
      var_g = var_1 ;
      var_b = V     ;
    }
    else                   {
      var_r = V     ;
      var_g = var_1 ;
      var_b = var_2 ;
    }

    R = (1-var_r) * 255;                  //RGB results = 0 ÷ 255
    G = (1-var_g) * 255;
    B = (1-var_b) * 255;
  }
}

void swapDisplay(void) {
  while(currentRow);
  memcpy((uint8_t *)displayArray, newDisplayArray, 3 * 32);
  return;
}

void clearPixels(void) {
  for(uint8_t i = 0; i < 32; i++) {
    newDisplayArray[0][i] = 0;
    newDisplayArray[1][i] = 0;
    newDisplayArray[2][i] = 0;
  }
}

void setPixel(uint8_t column, uint8_t row, uint8_t red, uint8_t green, uint8_t blue) {
  uint8_t colOut = 0;
  switch(column) {
  case 0:
    colOut = 16;
    break;
  case 1:
    colOut = 19;
    break;
  case 2:
    colOut = 22;
    break;
  case 3:
    colOut = 25;
    break;
  case 4:
    colOut = 28;
    break;
  case 5:
    colOut = 0;
    break;
  case 6:
    colOut = 3;
    break;
  case 7:
    colOut = 6;
    break;
  case 8:
    colOut = 9;
    break;
  default:
    break;
  }

  newDisplayArray[row][(colOut)] = green;
  newDisplayArray[row][(colOut) + 1] = red;
  newDisplayArray[row][(colOut) + 2] = blue;
}

void SPISetup(void)
{
  SPCR = ((1<<SPE)|               // SPI Enable
  (0<<SPIE)|              // SPI Interupt Enable
  (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
  (1<<MSTR)|              // Master/Slave select
  (0<<SPR1)|(0<<SPR0)|    // SPI Clock Rate
  (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
  (0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)
  SPSR = (1 << SPI2X);

  SPISSOn();
}

void SPISSOn(void)
{
  sbi(PORTB, 2);
}

void SPISSOff(void)
{
  cbi(PORTB, 2);
}

void SPIWriteByte(uint8_t dataByte)
{
  SPDR = dataByte;
  while((SPSR & (1<<SPIF))==0);
}

uint8_t checkDifference(uint8_t itemOne, uint8_t itemTwo, uint8_t threshold) {
  uint8_t diff;

  if(itemOne > itemTwo) {
    diff = itemOne - itemTwo;
  } 
  else {
    diff = itemTwo - itemOne;
  }

  if(diff >= threshold) {
    return 1;
  } 
  else {
    return 0;
  }
}

