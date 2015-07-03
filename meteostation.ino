#include <LiquidCrystal_I2C.h>// библиотеки для дисплея
#include <Wire.h>
#include <SFE_BMP180.h>


#define PRESSURE_MEASUREMENT_INTERVAL 2000
#define LED 13 // LED на D13
#define CHANNEL 3

LiquidCrystal_I2C lcd(0x27, 16, 2);
SFE_BMP180 pressure;

// температура, влажность и батарейка
float t[2];
byte h[2];
byte b[2];

unsigned long dispNow = 0;

// Oregon V2 decoder added - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $

class DecodeOOK {
  protected:
    byte total_bits, bits, flip, state, pos, data[25];

    virtual char decode (word width) = 0;

  public:

    enum {
      UNKNOWN, T0, T1, T2, T3, OK, DONE
    };

    DecodeOOK () {
      resetDecoder();
    }

    bool nextPulse (word width) {
      if (state != DONE)

        switch (decode(width)) {
          case -1:
            resetDecoder();
            break;
          case 1:
            done();
            break;
        }
      return isDone();
    }

    bool isDone () const {
      return state == DONE;
    }

    const byte* getData (byte& count) const {
      count = pos;
      return data;
    }

    void resetDecoder () {
      total_bits = bits = pos = flip = 0;
      state = UNKNOWN;
    }

    // add one bit to the packet data buffer

    virtual void gotBit (char value) {
      total_bits++;
      byte *ptr = data + pos;
      *ptr = (*ptr >> 1) | (value << 7);

      if (++bits >= 8) {
        bits = 0;
        if (++pos >= sizeof data) {
          resetDecoder();
          return;
        }
      }
      state = OK;
    }

    // store a bit using Manchester encoding
    void manchester (char value) {
      flip ^= value; // manchester code, long pulse flips the bit
      gotBit(flip);
    }

    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max = 0) {
      // align bits
      if (bits != 0) {
        data[pos] >>= 8 - bits;
        for (byte i = 0; i < pos; ++i)
          data[i] = (data[i] >> bits) | (data[i + 1] << (8 - bits));
        bits = 0;
      }
      // optionally shift bytes down if there are too many of 'em
      if (max > 0 && pos > max) {
        byte n = pos - max;
        pos = max;
        for (byte i = 0; i < pos; ++i)
          data[i] = data[i + n];
      }
    }

    void reverseBits () {
      for (byte i = 0; i < pos; ++i) {
        byte b = data[i];
        for (byte j = 0; j < 8; ++j) {
          data[i] = (data[i] << 1) | (b & 1);
          b >>= 1;
        }
      }
    }

    void reverseNibbles () {
      for (byte i = 0; i < pos; ++i)
        data[i] = (data[i] << 4) | (data[i] >> 4);
    }

    void done () {
      while (bits)
        gotBit(0); // padding
      state = DONE;
    }
};

// 433 MHz decoders


class OregonDecoderV2 :
  public DecodeOOK {
  public:
    OregonDecoderV2() {
    }

    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
      if (!(total_bits & 0x01))
      {
        data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
      }
      total_bits++;
      pos = total_bits >> 4;
      if (pos >= sizeof data) {
        resetDecoder();
        return;
      }
      state = OK;
    }

    virtual char decode (word width) {
      if (200 <= width && width < 1200) {
        byte w = width >= 700;
        switch (state) {
          case UNKNOWN:
            if (w != 0) {
              // Long pulse
              ++flip;
            }
            else if (32 <= flip) {
              // Short pulse, start bit
              flip = 0;
              state = T0;
            }
            else {
              // Reset decoder
              return -1;
            }
            break;
          case OK:
            if (w == 0) {
              // Short pulse
              state = T0;
            }
            else {
              // Long pulse
              manchester(1);
            }
            break;
          case T0:
            if (w == 0) {
              // Second short pulse
              manchester(0);
            }
            else {
              // Reset decoder
              return -1;
            }
            break;
        }
      }
      else {
        return -1;
      }
      return total_bits == 160 ? 1 : 0;
    }
};

OregonDecoderV2 orscV2;

volatile word pulse;

void ext_int_1(void) {
  static word last;
  // Определяем длину "пульса" в микросекундах, "for either polarity"
  pulse = micros() - last;
  last += pulse;
}


void reportLCD (const char* s, class DecodeOOK& decoder) {
  byte pos;
  const byte* data = decoder.getData(pos);
  
  if (data[0] == 0x1A && data[1] == 0x2D)
  {
    if (channel(data) == CHANNEL) { // Считываем данные с датчика, канал которого объявлен в начале кода
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.setCursor(5, 0);
      lcd.print(temperature(data)); // Температура
      lcd.setCursor(9, 0);
      lcd.print("'C");
      lcd.setCursor(0, 1);
      lcd.print("Hum:");         // Будет нечто вроде, где "_" - пустые клетки
      lcd.setCursor(4, 1);       //  _________________
      lcd.print(humidity(data)); // /________________/|
      lcd.setCursor(6, 1);       // |Temp:9.80'C_____||
      lcd.print("%");            // |Hum:50%_752.22mm|/
    }                            //
  }
  decoder.resetDecoder(); // Я не знаю, что это такое, но это должно быть
}
uint8_t percent[8] = {B11100, B10100, B11100, B00000, B00000, B00000, B00000, B00000};
long time;
void setup () {
  
  pressure.begin();
  // А как же без дисплея?
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Loading...");
  lcd.createChar(0, percent);
  pinMode(2, INPUT);  // Здесь подключен приемник
  digitalWrite(2, 1); // Подтягивающий резистор
  attachInterrupt(0, ext_int_1, CHANGE); // Типа прерывание(тоже нужно)
}
long previousTime; // Тайминг для датчика давления
void loop () {

  long currentTime = millis(); // Текущее время
  char status; // Тут все ясно
  double P, T; // И тут
  if (currentTime - previousTime > PRESSURE_MEASUREMENT_INTERVAL) { // Показания на экране обновляются раз в PRESSURE_MEASUREMENT_INTERVAL
    previousTime = currentTime;
    status = pressure.startTemperature();
    if (status != 0) {
      delay(status);
      status = pressure.getTemperature(T);
      if (status != 0) {
        status = pressure.startPressure(3);
        if (status != 0) {
          delay(status);
          status = pressure.getPressure(P, T);
          if (status != 0) {
            lcd.setCursor(8, 1);
            lcd.print(P * 0.75);
            lcd.setCursor(14, 1);
            lcd.print("mm");
          }
        }
      }
    }
  }
  noInterrupts();
  word p = pulse;


  pulse = 0;
  interrupts();

  if (p != 0) {
    if (orscV2.nextPulse(p)) {
      reportLCD("OSV2", orscV2);
      digitalWrite(LED, HIGH);
    }
  }

  

}

float temperature(const byte* data)
{
  int sign = (data[6] & 0x8) ? -1 : 1;
  float temp = ((data[5] & 0xF0) >> 4) * 10 + (data[5] & 0xF) + (float)(((data[4] & 0xF0) >> 4) / 10.0);
  return sign * temp;
}

byte humidity(const byte* data)
{
  return (data[7] & 0xF) * 10 + ((data[6] & 0xF0) >> 4);
}

// Ne retourne qu'un apercu de l'etat de la baterie : 10 = faible
byte battery(const byte* data)
{
  return (data[4] & 0x4) ? 10 : 90;
}

byte channel(const byte* data)
{
  byte channel;
  switch (data[2])
  {
    case 0x10:
      channel = 1;
      break;
    case 0x20:
      channel = 2;
      break;
    case 0x40:
      channel = 3;
      break;
  }
  return channel;
}

void printValues() {
  char buf[16];
  char tbuf[6];
  char templ[] = {' ', '%', 's', char(223), 'C', ' ', '%', '3', 'd', '%', '%', ' ', ' ', ' ', '\0'};
  for (int i = 0; i < 2; i++) {
    if (b[i] > 50) {
      templ[13] = char(219);
    }
    else {
      templ[13] = char(161);
    }
    dtostrf(t[i], 5, 1, tbuf);
    sprintf(buf, templ, tbuf, h[i]);
    Serial.println(buf);
  }
}

