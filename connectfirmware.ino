#include "OneWireHub.h"
#include "OneWirePole.h"
#include "OneWire.h"

#include "utils.h"

#include "LowLevel.h"
#include "CapTouch.h"

unsigned MY_INDEX = 100;
constexpr int NUM_POLES{20};

#include "SoftwareSerialWithHalfDuplex.h"

constexpr uint8_t SERIAL_RX_PIN{10};
constexpr uint8_t SERIAL_TX_PIN{11};

constexpr uint8_t OUT_SERIAL_RX_PIN{8};
constexpr uint8_t OUT_SERIAL_TX_PIN{9};

constexpr unsigned long SERIAL_RATE{4800};
constexpr unsigned long COMM_SERIAL_RATE{9600};

SoftwareSerialWithHalfDuplex mySerial(SERIAL_RX_PIN, SERIAL_TX_PIN, false, false);

const uint8_t WIRE_INDEXES[NUM_POLES] = {42, 25, 37, 91, 112,
                                         107, 56, 100, 62, 89,
                                         123, 29, 53, 71, 45,
                                         14, 77, 122, 86, 22};

constexpr uint8_t TOUCH_LED_INDEX{13};
constexpr uint8_t CONNECT_LED_INDEX{12};

constexpr uint8_t CAP_TX{4};
constexpr uint8_t CAP_RX{6};

constexpr uint8_t pin_onewire{7};

constexpr unsigned int CAP_SAMPLES{15};
constexpr unsigned int CAP_THRESHOLD{50};

constexpr unsigned int SILENCE_AFTER_READ{20};
constexpr unsigned int STATE_UPDATE{50};

CapTouch capTouch = CapTouch(CAP_TX, CAP_RX);

constexpr unsigned int TOUCH_TIMEOUT{500};

unsigned int TxPeriod() { return (100 + ((5 * MY_INDEX) % 50)); }

constexpr unsigned int CAP_SENSE_PERIOD{100};

// This is the ROM the arduino will respond to, make sure it doesn't conflict with another device

#define CONNECT_READ_STATE 0xBE

auto hub = OneWireHub(pin_onewire);
OneWirePole *pole = nullptr;

int getbit(int pin, int bit)
{
  if (digitalRead(pin) == LOW)
    return 1 << bit;
  else
    return 0;
}

bool UseOneWire = false;

SoftwareSerialWithHalfDuplex *CommSerial = nullptr;

void setup()
{
  if (MY_INDEX >= NUM_POLES)
  {
    pinMode(3, INPUT_PULLUP);
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    delay(1);
    MY_INDEX = getbit(3, 0) | getbit(A0, 1) | getbit(A1, 2) | getbit(A2, 3) | getbit(A3, 4);

    pinMode(3, INPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
  }

  if (UseOneWire)
  {
    pole = new OneWirePole(MY_INDEX);
  }
  else
  {
    CommSerial = new SoftwareSerialWithHalfDuplex(OUT_SERIAL_RX_PIN, OUT_SERIAL_TX_PIN, false, false);
  }

  pinMode(TOUCH_LED_INDEX, OUTPUT);
  pinMode(CONNECT_LED_INDEX, OUTPUT);

  Serial.begin(115200);
  Serial.print(MY_INDEX);
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

  if (UseOneWire)
  {
    hub.attach(*pole);
  }
  else
  {
    CommSerial->begin(COMM_SERIAL_RATE);
  }
  mySerial.begin(SERIAL_RATE);
  delay(500);
}

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

bool checkConnect()
{
  static uint8_t windowindex = 0;
  static uint8_t lastreceived = 0xff;

  static unsigned int lineindex = 0;
  unsigned long now = millis();
  bool gotSomething = false;

  while (mySerial.available() > 0)
  {
    uint8_t readbyte = mySerial.read();
    const uint8_t myindex = WIRE_INDEXES[MY_INDEX];
    const uint8_t mynotindex = ~myindex;

    if ((readbyte == 0xFF) || (readbyte == 0) || (readbyte == myindex) || (readbyte == mynotindex))
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
    int poleindex = -1;
    for (unsigned i = 0; i < COUNT_OF(WIRE_INDEXES); i++)
    {
      const uint8_t curindex = WIRE_INDEXES[i];
      const uint8_t curnotindex = ~curindex;

      if ((readbyte == curindex) || (readbyte == curnotindex))
      {
        poleindex = i;
        break;
      }
    }

    if (poleindex < 0)
    {
      return false;
    }
    if (poleindex >= NUM_POLES)
    {
      // impossible..
      return false;
    }

    //        Serial.print(lineindex++);
    //        Serial.print(" GOT INDEX ");
    //        Serial.println(poleindex);

    if (lastreceived != poleindex)
    {
      lastreceived = poleindex;
      windowindex = 1;
    }
    else
    {

      windowindex++;
      if (windowindex == 4)
      { //we saw 4 bytes!
        windowindex = 0;
        lastreceived = 0xff;

        Serial.print(lineindex++);
        Serial.print(" detect ");
        Serial.println(poleindex);
        poletimes[poleindex] = now + TOUCH_TIMEOUT;
      }
    }
  }
  return gotSomething;
}

void loop()
{

  static uint8_t windowindex = 0;
  static unsigned int lineindex = 0;
  static unsigned long silencedeadline = 0;
  static unsigned long stateupdate_deadline = 0;
  if (UseOneWire)
  {
    hub.poll();
  }

  bool gotSomething = checkConnect();

  unsigned long now = millis();
  if (gotSomething)
  {
    silencedeadline = now + SILENCE_AFTER_READ;
    //    Serial.println("GOT SOMETHINGGGGGGGG");
    return;
  }

  if (now > stateupdate_deadline)
  {
    stateupdate_deadline = now + STATE_UPDATE;
    copyState();
    if (UseOneWire)
    {
      hub.poll();
    }
  }

  //     Serial.println(" got nothing");

  if (now < silencedeadline)
  {
    return;
  }

  if (now > capdeadline)
  {
    if (UseOneWire)
    {
      hub.poll();
    }

    capdeadline = millis() + CAP_SENSE_PERIOD;

    //    FreqMeasure.end();
    mySerial.end();
    //    pinMode(freqmeasure_pin, INPUT);
    //    pinMode(signal_pin, INPUT);

    if (touchdetect())
    {
      // Serial.println("i'm touched");
      poletimes[MY_INDEX] = now + TOUCH_TIMEOUT;
    }
    pinMode(CAP_TX, INPUT);
    pinMode(CAP_RX, INPUT);
    //    FreqMeasure.begin();
    mySerial.begin(SERIAL_RATE);
  }

  if (now > txdeadline)
  {
    if (UseOneWire)
    {
      hub.poll();
    }
    txdeadline = millis() + TxPeriod();
    //    FreqMeasure.end();

    //    pinMode(freqmeasure_pin, INPUT);
    //    pinMode(signal_pin, INPUT);
    send_myself();
    //    pinMode(signal_pin, INPUT);
    //FreqMeasure.begin();
  }
}

void send_myself()
{

  uint8_t id = WIRE_INDEXES[MY_INDEX];
  uint8_t notid = ~id;

  for (int i = 0; i < 20; i++)
  {
    mySerial.write(&id, 1);
    mySerial.write(&notid, 1);
  }
}

void copyState()
{
  unsigned long now = millis();
  uint8_t currentstate[3] = {'\0'};

  static bool touching = false;
  bool istouch = false;
  bool isconnect = false;

  for (unsigned i = 0; i < COUNT_OF(poletimes); i++)
  {
    if (poletimes[i] > now)
    {
      currentstate[(i >> 3)] |= (1 << (i & 0x7));

      if (i == MY_INDEX)
      {
        istouch = true;
      }
      else
      {
        isconnect = true;
      }
    }
  }

  if (istouch != touching)
  {
    touching = istouch;
    if (touching)
    {
      Serial.println("i'm touched");
    }
    else
    {
      Serial.println("i'm NOT touched");
    }
  }

  digitalWrite(TOUCH_LED_INDEX, istouch ? HIGH : LOW);
  digitalWrite(CONNECT_LED_INDEX, isconnect ? HIGH : LOW);

  if (UseOneWire)
  {

    pole->copy_scrachpad(currentstate, COUNT_OF(currentstate));
  }
  else
  {
    uint8_t datatoSend[5] = {MY_INDEX, currentstate[0], currentstate[1], currentstate[2],0};
    datatoSend[COUNT_OF(datatoSend)-1] = OneWire::crc8(datatoSend, COUNT_OF(datatoSend)-1);
    CommSerial->write(datatoSend, COUNT_OF(datatoSend));
  }
}
