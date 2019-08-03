//
// UR5FFR Si5351 NanoVFO
// v1.0 from 2.04.2018
// Copyright (c) Andrey Belokon, 2017-2018 Odessa
// https://github.com/andrey-belokon/
// GNU GPL license
// Special thanks for
// Iambic sensor key http://www.jel.gr/cw-mode/iambic-keyer-with-arduino-in-5-minutes
// vk3hn CW keyer https://github.com/prt459/vk3hn_CW_keyer/blob/master/Basic_CW_Keyer.ino
//

#include <avr/power.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>


// !!! all user setting defined in config.h, config_hw.h and config_sw.h files !!!
#include "config.h"

#include "pins.h"
#include "i2c.h"
#include "TRX.h"
#include "Encoder.h"
#ifdef VFO_SI5351
#include "si5351a.h"
#endif
#ifdef VFO_SI570
#include "Si570.h"
#endif

#ifdef DISPLAY_MAX7219
#include "disp_MAX7219.h"
#endif
#ifdef DISPLAY_1602
#include "disp_1602.h"
#endif
#ifdef DISPLAY_OLED128x32
#include "disp_OLED128x32.h"
#endif
#ifdef DISPLAY_OLED128x64
#include "disp_OLED128x64.h"
#endif
#ifdef DISPLAY_OLED_SH1106_128x64
#include "disp_OLED_SH1106_128x64.h"
#endif

#ifdef VFO_SI5351
Si5351 vfo5351;
#endif
#ifdef VFO_SI570
Si570 vfo570;
#endif

Encoder encoder(ENCODER_PULSE_PER_TURN, ENCODER_FREQ_LO_STEP, ENCODER_FREQ_HI_STEP, ENCODER_FREQ_HI_LO_TRASH);
TRX trx;

#ifdef DISPLAY_MAX7219
Display_MAX7219 disp;
#endif
#ifdef DISPLAY_1602
Display_1602_I2C disp(I2C_ADR_DISPLAY_1602);
#endif
#ifdef DISPLAY_OLED128x32
Display_OLED128x32 disp;
#endif
#ifdef DISPLAY_OLED128x64
Display_OLED128x64 disp;
#endif
#ifdef DISPLAY_OLED_SH1106_128x64
Display_OLED_SH1106_128x64 disp;
#endif


enum EPins : uint8_t {
  D10 = 10,
  D11 = 11,
  D12 = 12,
  D13 = 13,
  PIN_OUT_TONE = D10,
  PIN_OUT_KEY = D11,
  PIN_OUT_PTT = D12,
  PIN_OUT_CW = D13
};


enum EStates : uint8_t {
  LOW_,
  HIGH_,
  OUT_KEY_ACTIVE_LEVEL = HIGH_,
  OUT_PTT_ACTIVE_LEVEL = HIGH_,
  OUT_CW_ACTIVE_LEVEL = HIGH_,
  DIT,
  DAH
};

constexpr EStates operator!(EStates state) {
  return (state == LOW_ ? HIGH_ : LOW_);
}

template<EStates>
struct TTag {
};

namespace GPIO {
template<EPins>
struct Out;

template<>
struct Out<D10> {
  inline void setup(void ) {
    DDRB |= 1 << DDB2;
  }

  inline void set( TTag<HIGH> ) noexcept {
    PORTB |= 1 << PORTB2;
  }

  inline void set( TTag<LOW> ) noexcept {
    PORTB &= ~(1 << PORTB2);
  }

  inline void toggle( void ) noexcept {
    PORTB ^= 1 << PORTB2;
  }
};

template<>
struct Out<D11> {
  inline void setup(void ) noexcept {
    DDRB |= 1 << DDB3;
  }

  inline void set( TTag<HIGH> ) noexcept {
    PORTB |= 1 << PORTB3;
  }

  inline void set( TTag<LOW> ) noexcept {
    PORTB &= ~(1 << PORTB3);
  }
};

}


InputPullUpPin inPTT(PIN_IN_PTT);

OutputBinPin outCW(PIN_OUT_CW, !OUT_CW_ACTIVE_LEVEL, OUT_CW_ACTIVE_LEVEL);
OutputBinPin outPTT(PIN_OUT_PTT, !OUT_PTT_ACTIVE_LEVEL, OUT_PTT_ACTIVE_LEVEL);
//OutputBinPin outKEY(PIN_OUT_KEY, !OUT_KEY_ACTIVE_LEVEL, OUT_KEY_ACTIVE_LEVEL);
GPIO::Out<PIN_OUT_KEY> outKEY;
GPIO::Out<PIN_OUT_TONE> outTONE;

InputAnalogKeypad keypad(PIN_ANALOG_KEYPAD, 6);

struct _Settings SettingsDef[SETTINGS_COUNT] = {
  SETTINGS_DATA
};

uint16_t EEMEM eeSettingsVersion = 0;
int EEMEM eeSettings[SETTINGS_COUNT] = {0};

int Settings[SETTINGS_COUNT] = {0};

void writeSettings()
{
  eeprom_write_block(Settings, eeSettings, sizeof(Settings));
}

void resetSettings()
{
  for (uint8_t j = 0; j < SETTINGS_COUNT; j++)
    Settings[j] = SettingsDef[j].def_value;
}

void readSettings()
{
  uint16_t ver;
  ver = eeprom_read_word(&eeSettingsVersion);
  if (ver == ((SETTINGS_VERSION << 8) | SETTINGS_COUNT))
    eeprom_read_block(Settings, eeSettings, sizeof(Settings));
  else {
    // fill with defaults
    resetSettings();
    writeSettings();
    ver = (SETTINGS_VERSION << 8) | SETTINGS_COUNT;
    eeprom_write_word(&eeSettingsVersion, ver);
  }
}


void vfo_set_freq(long f1, long f2, long f3)
{
#ifdef VFO_SI570
#ifdef VFO_SI5351
  vfo570.set_freq(f1);
  vfo5351.set_freq(f2, f3, 0);
#else
  // single Si570
  if (f1 != 0) vfo570.set_freq(f1);
  else vfo570.set_freq(f3);
#endif
#else
#ifdef VFO_SI5351
  vfo5351.set_freq(f1, f2, f3);
#endif
#endif
}

void UpdateFreq( void )
{
  uint8_t cwtx = trx.cwtx();

#ifdef MODE_DC
  vfo_set_freq(
    cwtx ? 0 : CLK0_MULT * trx.Freq,
    0,
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
  );
#endif

#ifdef MODE_DC_QUADRATURE
  vfo5351.set_freq_quadrature(
    cwtx ? 0 : trx.Freq,
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
  );
#endif

#ifdef MODE_SINGLE_IF
#if defined(SSBDetectorFreq_USB) && defined(SSBDetectorFreq_LSB)
  vfo_set_freq( // инверсия боковой - гетеродин сверху
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * (trx.Freq + (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]))),
    cwtx ? 0 : CLK1_MULT * (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT])),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ]) : 0
#endif
  );
#elif defined(SSBDetectorFreq_USB)
  long f = trx.Freq;
  if (trx.sideband == LSB) {
    f += (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) - f);
  }
  vfo_set_freq(
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * f,
    cwtx ? 0 : CLK1_MULT * (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : 0
#endif
  );
#elif defined(SSBDetectorFreq_LSB)
  long f = trx.Freq;
  if (trx.sideband == USB) {
    f += (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - f);
  }
  vfo_set_freq(
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * f,
    cwtx ? 0 : CLK1_MULT * (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ] : 0
#endif
  );
#else
#error You must define (SSBDetectorFreq_LSB+Settings[ID_LSB_SHIFT]) or/and (SSBDetectorFreq_USB+Settings[ID_USB_SHIFT])
#endif
#endif

#ifdef MODE_SINGLE_IF_RXTX
#if defined(SSBDetectorFreq_USB) && defined(SSBDetectorFreq_LSB)
  long f = CLK1_MULT * (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]));
  vfo_set_freq( // инверсия боковой - гетеродин сверху
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * (trx.Freq + (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]))),
    trx.TX ? 0 : f,
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ]) : (trx.TX ? f : 0)
#endif
  );
#elif defined(SSBDetectorFreq_USB)
  long f = trx.Freq;
  if (trx.sideband == LSB) {
    f += (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) - f);
  }
  vfo_set_freq(
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * f,
    trx.TX ? 0 : CLK1_MULT * (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : (trx.TX ? CLK1_MULT * (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : 0)
#endif
  );
#elif defined(SSBDetectorFreq_LSB)
  long f = trx.Freq;
  if (trx.sideband == USB) {
    f += (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - f);
  }
  vfo_set_freq(
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    CLK0_MULT * f,
    trx.TX ? 0 : CLK1_MULT * (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ] : (trx.TX ? CLK1_MULT * (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) : 0)
#endif
  );
#else
#error You must define (SSBDetectorFreq_LSB+Settings[ID_LSB_SHIFT]) or/and (SSBDetectorFreq_USB+Settings[ID_USB_SHIFT])
#endif
#endif

#ifdef MODE_SINGLE_IF_SWAP
#if defined(SSBDetectorFreq_USB) && defined(SSBDetectorFreq_LSB)
  long vfo = CLK0_MULT * (trx.Freq + (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT])));
  long f = CLK1_MULT * (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]));
  vfo_set_freq( // инверсия боковой - гетеродин сверху
    cwtx ? 0 : (trx.TX ? f : vfo),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    trx.TX ? vfo : f,
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (trx.sideband == LSB ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ]) : 0
#endif
  );
#elif defined(SSBDetectorFreq_USB)
  long f = trx.Freq;
  if (trx.sideband == LSB) {
    f += (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) - f);
  }
  vfo_set_freq(
    cwtx ? 0 : (trx.TX ? CLK1_MULT * (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) : CLK0_MULT * f),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    trx.TX ? CLK0_MULT*f : CLK1_MULT * (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_USB + Settings[ID_USB_SHIFT]) + Settings[ID_CW_TONE_HZ] : 0
#endif
  );
#elif defined(SSBDetectorFreq_LSB)
  long f = trx.Freq;
  if (trx.sideband == USB) {
    f += (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]);
  } else {
    f = abs((SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - f);
  }
  vfo_set_freq(
    cwtx ? 0 : (trx.TX ? CLK1_MULT * (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) : CLK0_MULT * f),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? 0 :
#endif
    trx.TX ? CLK0_MULT*f : CLK1_MULT * (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]),
#ifdef CWTX_DIRECT_FREQ
    cwtx ? trx.Freq + (trx.sideband == LSB ? -Settings[ID_CW_TONE_HZ] : Settings[ID_CW_TONE_HZ]) : 0
#else
    cwtx ? (SSBDetectorFreq_LSB + Settings[ID_LSB_SHIFT]) - Settings[ID_CW_TONE_HZ] : 0
#endif
  );
#else
#error You must define (SSBDetectorFreq_LSB+Settings[ID_LSB_SHIFT]) or/and (SSBDetectorFreq_USB+Settings[ID_USB_SHIFT])
#endif
#endif
}



void UpdateBandCtrl()
{
#ifdef BAND_ACTIVE_LEVEL_HIGH
  if (BAND_COUNT <= 4) {
    digitalWrite(PIN_OUT_BAND0, trx.BandIndex == 0);
    digitalWrite(PIN_OUT_BAND1, trx.BandIndex == 1);
    digitalWrite(PIN_OUT_BAND2, trx.BandIndex == 2);
    digitalWrite(PIN_OUT_BAND3, trx.BandIndex == 3);
  } else {
    digitalWrite(PIN_OUT_BAND0, trx.BandIndex & 0x1);
    digitalWrite(PIN_OUT_BAND1, trx.BandIndex & 0x2);
    digitalWrite(PIN_OUT_BAND2, trx.BandIndex & 0x4);
    digitalWrite(PIN_OUT_BAND3, trx.BandIndex & 0x8);
  }
#else
  if (BAND_COUNT <= 4) {
    digitalWrite(PIN_OUT_BAND0, trx.BandIndex != 0);
    digitalWrite(PIN_OUT_BAND1, trx.BandIndex != 1);
    digitalWrite(PIN_OUT_BAND2, trx.BandIndex != 2);
    digitalWrite(PIN_OUT_BAND3, trx.BandIndex != 3);
  } else {
    digitalWrite(PIN_OUT_BAND0, !(trx.BandIndex & 0x1));
    digitalWrite(PIN_OUT_BAND1, !(trx.BandIndex & 0x2));
    digitalWrite(PIN_OUT_BAND2, !(trx.BandIndex & 0x4));
    digitalWrite(PIN_OUT_BAND3, !(trx.BandIndex & 0x8));
  }
#endif
}

struct morse_char_t {
  char ch[7];
};

morse_char_t MorseCode[] = {
  {'A', '.', '-',  0,   0,   0,   0},
  {'B', '-', '.', '.', '.',  0,   0},
  {'C', '-', '.', '-', '.',  0,   0},
  {'D', '-', '.', '.',  0,   0,   0},
  {'E', '.',  0,   0,   0,   0,   0},
  {'F', '.', '.', '-', '.',  0,   0},
  {'G', '-', '-', '.',  0,   0,   0},
  {'H', '.', '.', '.', '.',  0,   0},
  {'I', '.', '.',  0,   0,   0,   0},
  {'J', '.', '-', '-', '-',  0,   0},
  {'K', '-', '.', '-',  0,   0,   0},
  {'L', '.', '-', '.', '.',  0,   0},
  {'M', '-', '-',  0,   0,   0,   0},
  {'N', '-', '.',  0,   0,   0,   0},
  {'O', '-', '-', '-',  0,   0,   0},
  {'P', '.', '-', '-', '.',  0,   0},
  {'Q', '-', '-', '.', '-',  0,   0},
  {'R', '.', '-', '.',  0,   0,   0},
  {'S', '.', '.', '.',  0,   0,   0},
  {'T', '-',  0,   0,   0,   0,   0},
  {'U', '.', '.', '-',  0,   0,   0},
  {'V', '.', '.', '.', '-',  0,   0},
  {'W', '.', '-', '-',  0,   0,   0},
  {'X', '-', '.', '.', '-',  0,   0},
  {'Y', '-', '.', '-', '-',  0,   0},
  {'Z', '-', '-', '.', '.',  0,   0},
  {'0', '-', '-', '-', '-', '-',  0},
  {'1', '.', '-', '-', '-', '-',  0},
  {'2', '.', '.', '-', '-', '-',  0},
  {'3', '.', '.', '.', '-', '-',  0},
  {'4', '.', '.', '.', '.', '-',  0},
  {'5', '.', '.', '.', '.', '.',  0},
  {'6', '-', '.', '.', '.', '.',  0},
  {'7', '-', '-', '.', '.', '.',  0},
  {'8', '-', '-', '-', '.', '.',  0},
  {'9', '-', '-', '-', '-', '.',  0},
  {'/', '-', '.', '.', '-', '.',  0},
  {'?', '.', '.', '-', '-', '.', '.'},
  {'.', '.', '-', '.', '-', '.', '-'},
  {',', '-', '-', '.', '.', '-', '-'},
  {0}
};

long last_event = 0;
long last_pool = 0;
uint8_t in_power_save = 0;
uint8_t last_key = 0;
long last_cw = 0;


enum {imIDLE, imDIT, imDAH};
uint8_t im_state = imIDLE;

void power_save(uint8_t enable)
{
  if (enable) {
    if (!in_power_save) {
      disp.setBright(Settings[ID_DISPLAY_BRIGHT_LOW]);
      in_power_save = 1;
    }
  } else {
    if (in_power_save) {
      disp.setBright(Settings[ID_DISPLAY_BRIGHT_HIGH]);
      in_power_save = 0;
    }
    last_event = millis();
  }
}

uint8_t readDit()
{
  uint8_t dit;
  digitalWrite(PIN_KEY_READ_STROB, HIGH);
  delayMicroseconds(Settings[ID_CW_TOUCH_THRESHOLD]);
  if (digitalRead(PIN_IN_DIT)) dit = 0; else dit = 1;
  digitalWrite(PIN_KEY_READ_STROB, LOW);
  if (dit) Serial.println("dit");
  return dit;
}

uint8_t readDah()
{
  uint8_t dah;
  digitalWrite(PIN_KEY_READ_STROB, HIGH);
  delayMicroseconds(Settings[ID_CW_TOUCH_THRESHOLD]);
  if (digitalRead(PIN_IN_DAH)) dah = 0; else dah = 1;
  digitalWrite(PIN_KEY_READ_STROB, LOW);
  if (dah) Serial.println("dah");
  return dah;
}
//////////////////////////////////////////////////////////////////////////////////

uint8_t OutputTone_state = 0;
uint8_t OutputTone_toggle = 0;

inline void OutputTone( uint8_t value ) noexcept
{
  if (value) {
    if (!OutputTone_state) {
      int prescalers[] = {1, 8, 64, 256, 1024};
      uint8_t ipresc = 5, mreg;
      for (uint8_t i = 0; i <= 4; i++) {
        long t = F_CPU / (prescalers[i] * value * 2) - 1;
        if (t <= 0xFF) {
          ipresc = i;
          mreg = t;
          break;
        }
      }
      if (ipresc > 4) return;
      ipresc++;
      // init timer2 2kHz interrup
      cli();
      TCCR2A = 0;// set entire TCCR2A register to 0
      TCCR2B = 0;// same for TCCR2B
      TCNT2  = 0;//initialize counter value to 0
      // set compare match register for 2khz increments
      OCR2A = mreg;
      // turn on CTC mode
      TCCR2A |= (1 << WGM21);
      TCCR2B |= ipresc;
      // enable timer compare interrupt
      TIMSK2 |= (1 << OCIE2A);
      sei();
      OutputTone_state = 1;
    }
  } else {
    if (OutputTone_state) {
      // disable interrup
      cli();
      TIMSK2 = 0;
      sei();
      // set pin to zero
      OutputTone_state = 0;
    }
  }
}

/*
  void sendDit()
  {
  outKEY.Write(1);
  OutputTone( Settings[ID_CW_TONE_HZ] );
  delay(trx.dit_time);
  outKEY.Write(0);
  OutputTone( uint8_t{} );
  delay(trx.dit_time);
  last_cw = millis();
  }

  void sendDah()
  {
  outKEY.set(TTag<HIGH_> {});
  OutputTone( Settings[ID_CW_TONE_HZ]);
  delay(trx.dah_time);
  outKEY.set(TTag<LOW_> {});
  OutputTone( uint8_t{} );
  delay(trx.dit_time);
  last_cw = millis();
  }
*/

struct TCWKey {
    template<EStates dit_dah>
    void send( void ) {
      outKEY.set(TTag<HIGH_> {});
      OutputTone( Settings[ID_CW_TONE_HZ]);
      //delay(trx.dah_time);
      Delay(TTag<dit_dah> {});
      outKEY.set(TTag<LOW_> {});
      OutputTone( uint8_t{} );
      Delay(TTag<dit_dah> {});
      last_cw = millis();
    }
  private:
    void Delay( TTag<DIT> ) noexcept {
      delay_ms(trx.dit_time);
    }

    void Delay( TTag<DAH> ) noexcept {
      delay_ms(trx.dah_time);
    }


    void delay_ms(unsigned long ms)
    {
      uint32_t start = micros();

      while (ms > 0) {
        yield();
        while ( ms > 0 && (micros() - start) >= 1000) {
          ms--;
          start += 1000;
        }
      }
    }
};

TCWKey cw_key;

ISR(TIMER2_COMPA_vect)
{
  //digitalWrite(OutputTone_pin, OutputTone_toggle++ & 1);
  outTONE.toggle();
}


uint8_t readCWSpeed()
{
  return trx.setCWSpeed(
           Settings[ID_CW_SPEED_MIN] + (long)analogRead(PIN_CW_SPEED_POT) * (Settings[ID_CW_SPEED_MAX] - Settings[ID_CW_SPEED_MIN] + 1) / 1024,
           Settings[ID_CW_DASH_LEN]
         );
}

void playMessage(char* msg)
{
  char ch;
  while ((ch = *msg++) != 0) {
    if (ch == ' ')
      delay(trx.dit_time * Settings[ID_CW_WORD_SPACE] / 10);
    else {
      for (int i = 0; MorseCode[i].ch[0] != 0; i++)  {
        if (ch == MorseCode[i].ch[0]) {
          // play letter
          for (uint8_t j = 1; j < 7; j++) {
            switch (MorseCode[i].ch[j]) {
              case '.':
                cw_key.send<DIT>();
                break;
              case '-':
                cw_key.send<DAH>();
                break;
            }
          }
          delay(trx.dit_time * Settings[ID_CW_LETTER_SPACE] / 10);
          break;
        }
      }
    }
    if (readDit() || readDah()) return;
    readCWSpeed();
  }
}



void TRX::cwTXOn( void ) noexcept
{
  return;
  CWTX = 1;
  if (!TX) {
    TX = 1;
    outPTT.Write(1);
    UpdateFreq();
    disp.Draw(trx);
  }
}

void edit_item(uint8_t mi)
{
  int val = Settings[mi];
  if (SettingsDef[mi].id == 99) {
#ifdef VFO_SI5351
    vfo5351.out_calibrate_freq();
#endif
#ifdef VFO_SI570
    vfo570.out_calibrate_freq();
#endif
  }
  disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, val, 1);
  keypad.waitUnpress();
  while (readDit() || readDah()) ;
  while (1) {
    uint8_t key = keypad.Read();
    if (key == 0 && readDit()) key = 1;
    if (key == 0 && readDah()) key = 2;
    switch (key) {
      case 1:
        // save value
        Settings[mi] = val;
        writeSettings();
#ifdef VFO_SI5351
        if (SettingsDef[mi].id == 99) vfo5351.set_xtal_freq((SI5351_CALIBRATION / 10000) * 10000 + Settings[ID_SI5351_XTAL]);
#endif
        return;
        break;
      case 2:
        // exit
        return;
    }
    long d = encoder.GetDelta();
    if (d < 0) {
      val -= SettingsDef[mi].step;
      if (val < SettingsDef[mi].min_value) val = SettingsDef[mi].min_value;
      disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, val, 1);
    } else if (d > 0) {
      val += SettingsDef[mi].step;
      if (val > SettingsDef[mi].max_value) val = SettingsDef[mi].max_value;
      disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, val, 1);
    }
  }
}

void show_menu()
{
  uint8_t mi = 0;
  disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, Settings[mi], 0);
  keypad.waitUnpress();
  while (readDit() || readDah()) ;
  while (1) {
    uint8_t key = keypad.Read();
    if (key == 0 && readDit()) key = 1;
    if (key == 0 && readDah()) key = 2;
    switch (key) {
      case 1:
        if (SettingsDef[mi].id == 0) {
          resetSettings();
          writeSettings();
        } else // edit value
          edit_item(mi);
        disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, Settings[mi], 0);
        keypad.waitUnpress();
        while (readDit() || readDah()) ;
        break;
      case 2:
        // exit
        disp.clear();
        return;
        break;
    }
    long d = encoder.GetDelta();
    if (d < 0) {
      if (mi == 0) mi = SETTINGS_COUNT - 1;
      else mi--;
      disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, Settings[mi], 0);
    } else if (d > 0) {
      mi++;
      if (mi >= SETTINGS_COUNT) mi = 0;
      disp.DrawMenu(SettingsDef[mi].id, SettingsDef[mi].title, Settings[mi], 0);
    }
  }
}

void setup()
{
  //clock_prescale_set(clock_div_2);
  //Serial.begin(9600);
  readSettings();
  outCW.setup();
  outPTT.setup();
  outKEY.setup();
  inPTT.setup();
  keypad.setup();
  pinMode(PIN_OUT_BAND0, OUTPUT);
  pinMode(PIN_OUT_BAND1, OUTPUT);
  pinMode(PIN_OUT_BAND2, OUTPUT);
  pinMode(PIN_OUT_BAND3, OUTPUT);
  //pinMode(PIN_OUT_TONE, OUTPUT);
  outTONE.setup();
  pinMode(PIN_CW_SPEED_POT, INPUT);
  pinMode(PIN_KEY_READ_STROB, OUTPUT);
  digitalWrite(PIN_KEY_READ_STROB, LOW);
  pinMode(PIN_IN_DIT, INPUT);
  pinMode(PIN_IN_DAH, INPUT);
  i2c_init();
#ifdef VFO_SI5351
  // change for required output level
  vfo5351.setup(
    SI5351_CLK0_DRIVE,
    SI5351_CLK1_DRIVE,
    SI5351_CLK2_DRIVE
  );
  vfo5351.set_xtal_freq((SI5351_CALIBRATION / 10000) * 10000 + Settings[ID_SI5351_XTAL]);
#endif
#ifdef VFO_SI570
  vfo570.setup(SI570_CALIBRATION);
#endif
  encoder.setup();
  disp.setup();
  /*if ( (keypad.Read() == 1) && ((readDit() && !readDah()) || (!readDit() && readDah()))) {
    // show menu on startup
    show_menu();
    keypad.waitUnpress();
    while (readDit() || readDah()) ;
    }*/

}


void loop()
{

  if (millis() - last_pool > POOL_INTERVAL) {
    /*  last_pool = millis();
      uint8_t key = keypad.Read();
      if (!last_key && key) {
        // somewhat pressed
        last_key = key;
        switch (key) {
          case 1:
            trx.NextBand();
            break;
          case 2:
            trx.CW = !trx.CW;
            outCW.Write(trx.CW);
            break;
          case 3:
            show_menu();
            keypad.waitUnpress();
            break;
          case 4:
          case 5:
          case 6:
            // MEMO
            if (trx.CW) {
              cwTXOn();
              switch (key) {
                case 4:
                  playMessage(MEMO1);
                  break;
                case 5:
                  playMessage(MEMO2);
                  break;
                case 6:
                  playMessage(MEMO3);
                  break;
              }
            }
            break;
        }
        power_save(0);
      } else if (!key) {
        last_key = 0;
      }
    */
    long delta = encoder.GetDelta();
    if (delta) {
      //disp.DrawMenu(SettingsDef[0].id, SettingsDef[0].title, delta, 1);
      trx.ChangeFreq(delta);
      power_save(0);
    }

    readCWSpeed();

    //trx.TX = trx.CWTX || inPTT.Read();
    //outPTT.Write(trx.TX);

    UpdateFreq();
    UpdateBandCtrl();
    disp.Draw(trx);

    if (Settings[ID_POWER_DOWN_DELAY] > 0 && millis() - last_event > Settings[ID_POWER_DOWN_DELAY] * 1000)
      power_save(1);
  }

  if (trx.tx() || Settings[ID_CW_VOX]) {
    if (Settings[ID_CW_IAMBIC]) {
      if (im_state == imIDLE) {
        if (readDit()) im_state = imDIT;
        else if (readDah()) im_state = imDAH;
      }
      if (im_state == imDIT) {
        trx.cwTXOn();
        cw_key.send<DIT>();
        //now, if dah is pressed go there, else check for dit
        if (readDah()) im_state = imDAH;
        else {
          if (readDit()) im_state = imDIT;
          else {
            //delay(trx.dit_time*Settings[ID_CW_LETTER_SPACE]/10-trx.dit_time);
            im_state = imIDLE;
          }
        }
      } else if (im_state == imDAH) {
        trx.cwTXOn();
        cw_key.send<DAH>();
        //now, if dit is pressed go there, else check for dah
        if (readDit()) im_state = imDIT;
        else {
          if (readDah()) im_state = imDAH;
          else {
            //delay(trx.dit_time*Settings[ID_CW_LETTER_SPACE]/10-trx.dit_time);
            im_state = imIDLE;
          }
        }
      }
    } else {
      if (readDit()) {
        trx.cwTXOn();
        cw_key.send<DIT>();
      } else if (readDah()) {
        trx.cwTXOn();
        cw_key.send<DAH>();
      }
    }
  }

  if (trx.CWTX && millis() - last_cw > Settings[ID_BREAK_IN_DELAY]) {
    
    trx.CWTX = 0;
    //trx.TX = inPTT.Read();
    outPTT.Write(trx.tx());
    if (!trx.tx())
      UpdateFreq();
  }
}
