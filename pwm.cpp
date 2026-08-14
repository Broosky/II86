/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Project: Triggy                                                                                                         //
// Author: Jeffrey Bednar                                                                                                  //
// Copyright (c) Illusion Interactive, 2011 - 2026.                                                                        //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Headers/pwm.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Globals:
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t ubStarted = false;
uint8_t ubChannelCount = 0;
volatile uint8_t ubPwmCounter = 0;
SOFTWARE_PWM_CHANNEL_T softwarePwmChannels[SOFTWARE_PWM_MAX_CHANNELS];
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start Timer2 interrupt.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 16 MHz / 8 / 64 = 31.25 kHz interrupt rate. 31,250 / 256 = ~122 Hz PWM.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ATmega328P Timer Overview:
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                         ATmega328P
//                              |
//              +---------------+---------------+
//              |               |               |
//           Timer0          Timer1          Timer2
//            8-bit          16-bit           8-bit
//              |               |               |
//        +-----+-----+     +---+---+     +-----+-----+
//        |           |     |       |     |           |
//       D5          D6    D9      D10   D3          D11
//        |           |     |       |     |           |
//       PWM         PWM   PWM     PWM   PWM         PWM
//
// Timer0
// -------
// 8-bit timer.
//
// Responsible for:
//   - millis().
//   - micros().
//   - delay().
//   - Arduino timekeeping.
//
// Hardware PWM:
//   - D5 (OC0B)
//   - D6 (OC0A)
//
// Timer1
// -------
// 16-bit timer.
//
// Responsible for:
//   - Servo library.
//   - High-resolution timing.
//   - 16-bit PWM.
//
// Hardware PWM:
//   - D9  (OC1A)
//   - D10 (OC1B)
//
// Timer2
// -------
// 8-bit timer.
//
// Responsible for:
//   - tone().
//   - 8-bit PWM.
//
// Hardware PWM:
//   - D3  (OC2B)
//   - D11 (OC2A)
//
// Software PWM:
//   - Timer2 is used as the timebase for the software PWM.
//   - The Timer2 interrupt periodically updates non-PWM pins.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PWM Pin Summary:
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware PWM pins:
//
//   D3  -> Timer2 / OC2B
//   D5  -> Timer0 / OC0B
//   D6  -> Timer0 / OC0A
//   D9  -> Timer1 / OC1A
//   D10 -> Timer1 / OC1B
//   D11 -> Timer2 / OC2A
//
// Non-PWM digital pins:
//
//   D0
//   D1
//   D2
//   D4
//   D7
//   D8
//   D12
//   D13
//
// These pins do not have hardware PWM capability. They can be controlled by the software PWM implementation.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Software PWM Architecture:
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Timer2
//                            |
//                     Compare Interrupt
//                            |
//                     Software PWM ISR
//                            |
//              +-------------+-------------+
//              |             |             |
//             PORTB         PORTC         PORTD
//              |             |             |
//         D8, D12, D13     A0-A5       D2, D4, D7
//                                          |
//                                     Software PWM
//
// The ISR directly manipulates the AVR PORT registers to generate PWM on pins that do not have hardware PWM capability.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void softwarePwmBegin(void) {
  if (ubStarted) {
    return;
  }

  uint8_t ubSReg = SREG;

  cli();

  TCCR2A = _BV(WGM21);  // CTC mode.
  TCCR2B = _BV(CS21);   // prescaler = 8.

  OCR2A = 63;

  TIMSK2 |= _BV(OCIE2A);

  ubStarted = true;

  SREG = ubSReg;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sets the software PWM duty cycle for a software PWM pin.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void softwarePwmAnalogWrite(uint8_t ubPin, uint8_t ubValue) {
  int8_t nIndex;
  uint8_t ubPort, ubMask;
  volatile uint8_t* p_ubOut;

  // Start the software PWM engine if we haven't already.
  softwarePwmBegin();

  ubPort = digitalPinToPort(ubPin);

  if (ubPort == NOT_A_PIN) {
    return;
  }

  p_ubOut = portOutputRegister(ubPort);
  ubMask = digitalPinToBitMask(ubPin);

  // See if this pin is already being controlled.
  nIndex = softwarePwmFindChannel(ubPin);

  if (nIndex >= 0) {
    softwarePwmChannels[nIndex].ubDutyCycle = ubValue;

    // Handle endpoints immediately.
    if (ubValue == 0) {
      *p_ubOut &= (uint8_t)~ubMask;
    } else if (ubValue == 255) {
      *p_ubOut |= ubMask;
    }

    return;
  }

  // Add a new channel if we can.
  if (ubChannelCount >= SOFTWARE_PWM_MAX_CHANNELS) {
    return;
  } else {
    uint8_t ubSReg = SREG;

    cli();

    pinMode(ubPin, OUTPUT);

    softwarePwmChannels[ubChannelCount].p_ubPort = p_ubOut;
    softwarePwmChannels[ubChannelCount].ubMask = ubMask;
    softwarePwmChannels[ubChannelCount].ubDutyCycle = ubValue;

    // Set an initial output state.
    if (ubValue == 0)
      *p_ubOut &= (uint8_t)~ubMask;
    else
      *p_ubOut |= ubMask;

    ubChannelCount++;

    SREG = ubSReg;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Find an existing software PWM channel.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int8_t softwarePwmFindChannel(uint8_t ubPin) {
  uint8_t ubPort = digitalPinToPort(ubPin);

  if (ubPort == NOT_A_PIN) {
    return -1;
  }

  volatile uint8_t* p_ubOut = portOutputRegister(ubPort);
  uint8_t ubMask = digitalPinToBitMask(ubPin);

  for (uint8_t ubI = 0; ubI < ubChannelCount; ubI++) {
    if (softwarePwmChannels[ubI].p_ubPort == p_ubOut && softwarePwmChannels[ubI].ubMask == ubMask) {
      return ubI;
    }
  }

  return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer2 software PWM interrupt service routine.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) {
  uint8_t ubCounter = ++ubPwmCounter;

  for (uint8_t ubI = 0; ubI < ubChannelCount; ubI++) {
    if (ubCounter < softwarePwmChannels[ubI].ubDutyCycle) {
      *softwarePwmChannels[ubI].p_ubPort |= softwarePwmChannels[ubI].ubMask;
    } else {
      *softwarePwmChannels[ubI].p_ubPort &= (uint8_t)~softwarePwmChannels[ubI].ubMask;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////