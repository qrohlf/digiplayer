// Audio playback sketch for Adafruit Trinket.  Requires 3.3V
// Trinket board and Winbond W25Q80BV serial flash loaded with
// audio data.  PWM output on pin 4; add ~25 KHz low-pass filter
// prior to amplification.  Uses ATtiny-specific registers;
// WILL NOT RUN ON OTHER ARDUINOS.

//#include <Adafruit_TinyFlash.h>
//#include <avr/io.h>
#include <avr/pgmspace.h>
#include <core_timers.h>
#include "fbnotify.h"

uint16_t           delay_count;
volatile uint32_t  index = 0L;
char donePlaying = false;

void stopPlaying() {
  //stop sending PWM
  //pinMode(4, INPUT);
  //reset registers to defaults
  Timer0_SetToPowerup();
  Timer0_SetWaveformGenerationMode(Timer0_Normal);
  Timer0_ClockSelect(Timer0_Prescale_Value_1024);
  Timer1_SetToPowerup();
  Timer1_SetWaveformGenerationMode(Timer1_Normal);
  Timer1_ClockSelect(Timer1_Prescale_Value_1024);
  donePlaying = true;
}

void startPlaying() {
  uint8_t  data[6];
  uint32_t bytes;
  donePlaying = false;

  PLLCSR |= _BV(PLLE);               // Enable 64 MHz PLL
  delay(100);            // Stabilize
  while(!(PLLCSR & _BV(PLOCK)));     // Wait for it...
  PLLCSR |= _BV(PCKE);               // Timer1 source = PLL

  // Set up Timer/Counter1 for PWM output
  TIMSK  = 0;                        // Timer interrupts OFF
  TCCR1  = _BV(CS10);                // 1:1 prescale
  GTCCR  = _BV(PWM1B) | _BV(COM1B1); // PWM B, clear on match
  OCR1C  = 255;                      // Full 8-bit PWM cycle
  OCR1B  = 127;                      // 50% duty at start

  pinMode(4, OUTPUT);                // Enable PWM output pin

  // Set up Timer/Counter0 for sample-playing interrupt.
  // TIMER0_OVF_vect is already in use by the Arduino runtime,
  // so TIMER0_COMPA_vect is used.  This code alters the timer
  // interval, making delay(), micros(), etc. useless (the
  // overflow interrupt is therefore disabled).

  // Timer resolution is limited to either 0.125 or 1.0 uS,
  // so it's rare that the playback rate will precisely match
  // the data, but the difference is usually imperceptible.
  TCCR0A = _BV(WGM01) | _BV(WGM00);  // Mode 7 (fast PWM)
  if(sounddata_sampleRate >= 31250) {
    TCCR0B = _BV(WGM02) | _BV(CS00); // 1:1 prescale
    OCR0A  = ((F_CPU + (sounddata_sampleRate / 2)) / sounddata_sampleRate) - 1;
  } else {                           // Good down to about 3900 Hz
    TCCR0B = _BV(WGM02) | _BV(CS01); // 1:8 prescale
    OCR0A  = (((F_CPU / 8L) + (sounddata_sampleRate / 2)) / sounddata_sampleRate) - 1;
  }
  TIMSK = _BV(OCIE0A); // Enable compare match, disable overflow
}

ISR(TIMER0_COMPA_vect) {
  if (donePlaying == false) {
    OCR1B = pgm_read_byte(&sounddata_data[index]);
    if(++index > sounddata_length) {           // End of audio data?
        index = 0;                       // We must repeat!
        OCR1B  = 127;                      // 50% duty at start
        stopPlaying();
    }
  }
}

void setup() {
  startPlaying();
}

void loop() {
  int interval = 0;
  if (donePlaying) {
    interval = 1000000 + rand() % 6000000;
    /*if (rand()%3 == 2) interval = 10000;*/ //for some reason math doesn't work right now
    delay(interval);

    startPlaying();
  }
}
