#include<stdlib.h>
// RF 433
const byte TX_PIN = 9;
// SR04
const byte SR04_TRIG=12;
const byte SR04_ECHO=11;
long lecture_echo; // echo transmis par le SR04
double distance_cm;// distance mesurée par le capteur moins le vide nécessaire (h_vide)
int percent=0;//pourcentage d'occupation de fioul
const double ref_h=160; // ht maxi de liquide = long entre le niveau mini(fond de fioul généralement plein de sédiment) et avant la ht_vide 
int h_vide=35; // Vide nécessaire entre le capteur et le niveau maxi

#define LED_OUT 10
// LCD
#define PIN_SCE   7
#define PIN_RESET 6
#define PIN_DC    5
#define PIN_SDIN  4
#define PIN_SCLK  3
#define LCD_CMD   0
#define LCD_C     LOW
#define LCD_D     HIGH
#define LCD_X     84
#define LCD_Y     48

static const byte Digits[][4][18] = 
{
 {
    { 0xE0, 0xF0, 0xF8, 0xF4, 0xEE, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x1F, 0x3F, 0x7F, 0x3F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0x3F, 0x1F },  
    { 0xFC, 0xFE, 0xFF, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x03, 0x07, 0x0F, 0x17, 0x3B, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  },
  {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF8, 0xF0, 0xE0 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0x3F, 0x1F },  
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x07, 0x03 },
  },
  {
    { 0x00, 0x00, 0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F },  
    { 0xFC, 0xFE, 0xFF, 0xFE, 0xFD, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00 }, 
    { 0x03, 0x07, 0x0F, 0x17, 0x3B, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x38, 0x10, 0x00, 0x00, 0x00 },
  },
  {
    { 0x00, 0x00, 0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F },  
    { 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  },
  {
    { 0xE0, 0xF0, 0xF8, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF8, 0xF0, 0xE0 },
    { 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F },  
    { 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x07, 0x03 },
  },
  {
    { 0xE0, 0xF0, 0xF8, 0xF4, 0xEE, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00, 0x00 },
    { 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00 },  
    { 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  },
  {
    { 0xE0, 0xF0, 0xF8, 0xF4, 0xEE, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00, 0x00 },
    { 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00 },  
    { 0xFC, 0xFE, 0xFF, 0xFE, 0xFD, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x03, 0x07, 0x0F, 0x17, 0x3B, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  },
  {
    { 0x00, 0x00, 0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0x3F, 0x1F },  
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x07, 0x03 },
  },
  {
    { 0xE0, 0xF0, 0xF8, 0xF4, 0xEE, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F },  
    { 0xFC, 0xFE, 0xFF, 0xFE, 0xFD, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x03, 0x07, 0x0F, 0x17, 0x3B, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  },
  {
    { 0xE0, 0xF0, 0xF8, 0xF4, 0xEE, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xEE, 0xF4, 0xF8, 0xF0, 0xE0 },
    { 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F },  
    { 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xFD, 0xFE, 0xFF, 0xFE, 0xFC }, 
    { 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x7C, 0x3B, 0x17, 0x0F, 0x07, 0x03 },
  }
};

int boucle=0;

 
const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;
 
#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)

byte OregonMessageBuffer[9];
 
/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) 
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}
 
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) 
{
   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}
 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
 byte POSTAMBLE[]={0x00};
 sendData(POSTAMBLE, 1);  
}
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel) 
{
    data[2] = channel;
}
 
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID) 
{
  data[3] = ID;
}
 
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setDistance(byte *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}
 
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setPercent(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
 
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}
 
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/


void readSR04(){
  digitalWrite(SR04_TRIG, HIGH); 
  delay(20); 
  digitalWrite(SR04_TRIG, LOW); 
  lecture_echo = pulseIn(SR04_ECHO, HIGH); 
  distance_cm = lecture_echo / 58.0;
  distance_cm =distance_cm -h_vide;
  if (distance_cm < 0){
    distance_cm=0;
  } 
  percent = (1-(distance_cm/ref_h))*100;
  Serial.print("d - Distance cm : "); 
  Serial.println(distance_cm); 
}

void LcdInitialise(void)
{
  pinMode(PIN_SCE, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  digitalWrite(PIN_RESET, HIGH);

  LcdWrite( LCD_CMD, 0x21 ); // LCD Extended Commands.
  LcdWrite( LCD_CMD, 0xC8 ); // Set LCD Vop (Contrast)
  LcdWrite( LCD_CMD, 0x06 ); // Set Temp coefficent
  LcdWrite( LCD_CMD, 0x14 ); // LCD bias mode 1:48

  LcdWrite( LCD_CMD, 0x20 ); // LCD Standard Commands.
  LcdWrite( LCD_CMD, 0x0C ); // LCD in normal mode. 0x0d for inverse
}

void LcdWrite(byte dc, byte data)
{
  digitalWrite(PIN_DC, dc);
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}

void LcdClear(void)
{
  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
  {
    LcdWrite(LCD_D, 0x00);
  }
}

void Spacer()
{
  LcdWrite(LCD_D, 0x00);
  LcdWrite(LCD_D, 0x00);
}

void DisplayValue(double value,int percent){
   int intValue= value*10.0;
   
   byte values[4] = 
  { 
    (byte)(intValue / 1000),            // 1536 > 1
    (byte)((intValue % 1000)/100),      // 536  > 5
    (byte)(((intValue % 1000)%100)/10), // 536  > 36 > 3
    (byte)(((intValue % 1000)%100)%10)  // 536  > 36 > 6
  };
    for(byte row = 0; row < 4; row++)
  {      
    LcdWrite(LCD_C, 0x80 | 0);
    LcdWrite(LCD_C, 0x40 | row);

    for(byte digit = 0; digit < 4; digit++)
    {
      for(byte col = 0; col < 18; col++)
      {        
        LcdWrite(LCD_D, Digits[values[digit]][row][col]);
      }      
      if(digit == 2){
        Spacer();
        }
      Spacer();
    }
  }
  int convertPercent = 0.82 * percent;
  DrawPercentBar(convertPercent);
  }

void DrawPercentBar(byte value)
{
  // Position the pointer
  LcdWrite(LCD_C, 0x80 | 0x00);
  LcdWrite(LCD_C, 0x45);

  // Draw the left side of the progress bar box
  LcdWrite(LCD_D, 0xFF);
  
  for(byte i = 0; i < 82; i++)
  {
    if(i < value)
    {
      LcdWrite(LCD_D, 0xFF);
    }
    else
    {
      LcdWrite(LCD_D, 0x81);
    }
  }

  // Draw the right side of the progress bar box  
  LcdWrite(LCD_D, 0xFF);
}


void initLCD(){
  LcdInitialise();
  LcdClear();
}

void sendRF(){
  digitalWrite(LED_OUT, HIGH);

  setBatteryLevel(OregonMessageBuffer, 1); // 0 : low, 1 : high
  setDistance(OregonMessageBuffer, distance_cm);
  setPercent(OregonMessageBuffer, percent);

 
  // Calculate the checksum
  calculateAndSetChecksum(OregonMessageBuffer);
 
  // Show the Oregon Message
  for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   {     Serial.print(OregonMessageBuffer[i] >> 4, HEX);
    Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
  }
  Serial.println();
  // Send the Message over RF
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(TWOTIME*8);
  // Send a copie of the first message. The v2.1 protocol send the
  // message two time 
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
 
  // Wait for 30 seconds before send a new message 
  SEND_LOW();
  delay(500); 
  digitalWrite(LED_OUT, LOW);
}


void setup()
{

  // INIT LCD
  initLCD();

  // INIT SR04
  pinMode(SR04_TRIG, OUTPUT); 
  digitalWrite(SR04_TRIG, LOW); 
  pinMode(SR04_ECHO, INPUT); 

  pinMode(LED_OUT, OUTPUT);
  digitalWrite(LED_OUT, HIGH);

  // INIT Console
  Serial.begin(115200);
  Serial.println("\n[Oregon V2.1 encoder]");
  
  // INIT RF 433
  pinMode(TX_PIN, OUTPUT);
  
  SEND_LOW();  
  byte ID[] = {0x1A,0x2D};
  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
  setId(OregonMessageBuffer, 0xBB);
 digitalWrite(LED_OUT, LOW);
}
 
void loop()
{
  readSR04();
  DisplayValue(distance_cm,percent);

  if (boucle==30||boucle==0){
    // Envoi message toutes les 30 boucles
    sendRF();
    boucle=0;
  }  
  delay(60000);
  boucle++;
}
