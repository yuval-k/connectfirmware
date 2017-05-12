
#include "LowLevel.h"
#include "OneWireSlave.h"
#include "CapTouch.h"

#include "SoftwareSerialWithHalfDuplex.h"

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define MY_INDEX 1

#define LED_INDEX 13

#define CAP_TX 5
#define CAP_RX 6
#define CAP_SAMPLES 15
#define CAP_THRESHOLD 100

CapTouch capTouch = CapTouch(CAP_TX, CAP_RX);

#define NUM_POLES 20
#define TOUCH_TIMEOUT 500

#define TX_PERIOD 60 + 20 * MY_INDEX

#define CAP_SENSE_PERIOD 100

#define ROM {0x28, MY_INDEX, 0x00, 0x00, 0x00, 0x00, 0x02}



enum DeviceState
{
  DS_WaitingReset,
  DS_WaitingCommand,
  DS_StateWritten,
};
volatile DeviceState state = DS_WaitingReset;

// scratchpad, with the CRC byte at the end
volatile byte scratchpad[9];

// This is the pin that will be used for one-wire data (depending on your arduino model, you are limited to a few choices, because some pins don't have complete interrupt support)
// On Arduino Uno, you can use pin 2 or pin 3
Pin oneWireData(2);

// This is the ROM the arduino will respond to, make sure it doesn't conflict with another device

#define CONNECT_READ_STATE 0xBE

SoftwareSerialWithHalfDuplex mySerial(10, 11, false, false);

void setup()
{

  pinMode(LED_INDEX, OUTPUT);

  Serial.begin(115200);
  mySerial.begin(38400);

  delay(500);
  
  
  Serial.println(" setup setup");


  // put your setup code here, to run once:
  // try to read 4 bytes from the serial.
  // if all 4 bytes are the same, mark it is being connect

  // if 4 bytes are different (or other error) ???

  // if after timeout, write our id to the serial for X amount of time.

  // sleep random amunt

  // write our id to the serial


  Serial.println("calibrating");
  capTouch.readTouch(CAP_SAMPLES);
  Serial.println("setting send pin back to input");
  pinMode(CAP_TX, INPUT);
    
  const byte owROM[7] = ROM;

 // OWSlave.setReceiveCallback(&owReceive);
//  OWSlave.begin(owROM, oneWireData.getPinNumber());

}

#define SEND_INDEX (MY_INDEX+1)

uint8_t myid[] = {SEND_INDEX, SEND_INDEX, SEND_INDEX, SEND_INDEX, SEND_INDEX};

unsigned long poletimes[NUM_POLES] = {0};

bool touchdetect()
{

  long res = capTouch.readTouch(CAP_SAMPLES);

 // Serial.print(" cap result      ");
 // Serial.println(res);

  return res > CAP_THRESHOLD;
}

unsigned long txdeadline = 0;
unsigned long capdeadline = 0;
unsigned long lastread = 0;
#define READTIMEOUT 1000

bool checkConnect() {
  static uint8_t windowindex = 0;
  
  uint8_t lastreceived = 0xff;
  
  static unsigned int lineindex = 0;
  unsigned long now = millis();
  bool gotSomething = false;
  while (mySerial.available() > 0)
  {
    byte readbyte = mySerial.read();
    if ((readbyte == 0) || (readbyte == SEND_INDEX) )
    {
      continue;
    }
    
    gotSomething = true;

    if (now > (lastread + READTIMEOUT))
    {
      lastreceived = 0xff;
      windowindex = 0;
    }

    
    lastread = now;
    // send index is +1..
    int poleindex = readbyte - 1;

    if (poleindex >= NUM_POLES) {
      // impossible..
      return false;
    }

    if (lastreceived != poleindex) {
      lastreceived = poleindex;
       windowindex = 1;
    } else {
      
      windowindex++;
      if (windowindex == 4) { //we saw 4 bytes!
        windowindex = 0;
        lastreceived = 0xff;
        
        Serial.print(lineindex++);
        Serial.print(" detect");
        Serial.println(poleindex);
        poletimes[poleindex] = now + TOUCH_TIMEOUT;
  
        copyState();
      }      
    }
  }
  return gotSomething;
  
}

void loop()
{
  
  static uint8_t windowindex = 0;
  
  uint8_t lastreceived = 0xff;
  
  static unsigned int lineindex = 0;
  
  bool gotSomething = checkConnect();

  digitalWrite(LED_INDEX, gotSomething);

  if (gotSomething)
  {
            Serial.println("GOT SOMETHINGGGGGGGG");

    return;
  }

   //     Serial.println(" got nothing");

        
  unsigned long now = millis();
  
  if (now > capdeadline)
  {
    capdeadline = now + CAP_SENSE_PERIOD;
    if (touchdetect())
    {
      poletimes[MY_INDEX] = now + TOUCH_TIMEOUT;
 Serial.println("I'm touched");

      copyState();
    }
    pinMode(CAP_TX, INPUT);
  }

  if (now > txdeadline)
  {

    txdeadline = now + TX_PERIOD;

    for (int i = 0; i < 2; i++)
    {
      mySerial.write(myid, COUNT_OF(myid));
    }
  }
}

void copyState()
{

  unsigned long now = millis();
  uint8_t currentstate[COUNT_OF(scratchpad)] = {'\0'};
  for (int i = 0; i < NUM_POLES; i++)
  {
    if (poletimes[i] < now)
    {
      currentstate[i >> 3] |= (1 << (i & 0x7));
    }
  }

  currentstate[COUNT_OF(currentstate)-1] = OWSlave.crc8(currentstate, COUNT_OF(currentstate)-1);

  cli();
  memcpy((void *)scratchpad, currentstate, COUNT_OF(currentstate));
  state = DS_StateWritten;
  sei();
}

void owReceive(OneWireSlave::ReceiveEvent evt, byte data)
{
  switch (evt)
  {
  case OneWireSlave::RE_Byte:
    switch (state)
    {
    case DS_WaitingCommand:
      switch (data)
      {
      case CONNECT_READ_STATE:
        state = DS_WaitingReset;
        OWSlave.beginWrite((const byte *)scratchpad, COUNT_OF(scratchpad), 0);
        break;
      }
      break;

    case OneWireSlave::RE_Reset:
      state = DS_WaitingCommand;
      break;

    case OneWireSlave::RE_Error:
      state = DS_WaitingReset;
      break;
    }
  }
}
