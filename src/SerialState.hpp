#ifndef SerialState_hpp
#define SerialState_hpp

#if defined(USBCON)

#include "USBAPI.h"
#include <util/atomic.h>

#define CDC_SERIAL_STATE 0x20

const union SERIAL_STATE_PACKET
{
  USBSetup packet;
  uint8_t buffer[sizeof(USBSetup)];
} SerialStatePacket PROGMEM = {REQUEST_DEVICETOHOST_CLASS_INTERFACE, CDC_SERIAL_STATE, 0, 0, CDC_ACM_INTERFACE, 2};

enum SERIAL_STATE : uint8_t
{
  SS_DCD = 0b1,
  SS_DSR = 0b10,
  SS_BREAK = 0b100,
  SS_RING = 0b1000,
  SS_FRAMING_ERROR = 0b10000,
  SS_PARITY_ERROR = 0b100000,
  SS_OVERRUN_ERROR = 0b1000000,
};

#define STATE_DCD 0
#define STATE_DSR 1
#define STATE_BREAK 2
#define STATE_RING 3
#define STATE_FRAMING_ERROR 4
#define STATE_PARITY_ERROR 5
#define STATE_OVERRUN_ERROR 6

volatile uint8_t ep;
volatile bool idle = false; //USB_EP_SIZE
volatile uint8_t _sreg;

inline void byteSendSerialData()
{
  if (!idle)
  {
    UEINTX = 0x3A; // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
    UENUM = ep;
    SREG = _sreg;
    idle = true;
  }
}

inline void byteSendSerialData(uint8_t data)
{
  if (idle)
  {
    cli();
    _sreg = SREG;

    ep = UENUM;
    UENUM = CDC_ENDPOINT_IN;
    while (!(UEINTX & (1 << TXINI)))
      ;
    UEINTX = ~(1 << TXINI);

    idle = false;
  }
  UEDATX = data;
  if (!(UEINTX & (1 << RWAL)))
  {
    UEINTX = 0x3A; // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
    UENUM = ep;
    idle = true;
    SREG = _sreg;
  }
}

bool sendSerialData(const uint8_t *data, uint8_t size)
{
  if (Serial)
  {
    uint8_t i = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      ep = UENUM;
      UENUM = CDC_ENDPOINT_IN;
      while (!(UEINTX & (1 << TXINI)))
        ;
      UEINTX = ~(1 << TXINI);

      for (; i < size; i++)
      {
        UEDATX = data[i];
      }

      UEINTX = 0x3A; // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
      UENUM = ep;
    }
    return 0;
  }
  return 1;
}

bool sendSerialState(uint8_t state)
{
  if (Serial)
  {
    uint8_t ep, i = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      ep = UENUM;
      UENUM = CDC_ENDPOINT_ACM;
      while (!(UEINTX & (1 << TXINI)))
        ;
      UEINTX = ~(1 << TXINI);

      for (; i < sizeof(USBSetup); i++)
      {
        UEDATX = pgm_read_byte(&SerialStatePacket.buffer[i]);
      }

      UEDATX = state;
      UEDATX = 0;

      UEINTX = 0x3A; // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
      UENUM = ep;
    }
    return 0;
  }
  return 1;
}

inline bool sendSerialState(SERIAL_STATE state)
{
  return sendSerialState(state);
}

#endif //#if defined(USBCON)

#endif //#ifndef SerialState_hpp