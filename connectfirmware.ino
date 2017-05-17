#include "OneWireHub.h"
#include "OneWirePole.h"

#include "utils.h"

#include "LowLevel.h"
#include "CapTouch.h"

int MY_INDEX = 4;
constexpr int NUM_POLES{20};


#ifdef USE_PWM

#include "FreqMeasure.h"
#include "TimerOne.h"

constexpr unsigned long freqmeasure_pin{8};
constexpr unsigned long signal_pin{9};

constexpr unsigned long FREQ_START{300};
constexpr unsigned long FREQ_END{1300};

constexpr unsigned long FREQ_SKIP{(FREQ_END - FREQ_START) / (NUM_POLES - 1)};
constexpr unsigned long FREQ_SENSE{FREQ_SKIP * 10 / 100}; // 10%

unsigned long SIGNAL_FREQ;
constexpr unsigned long signal_freq_for(int index) { return FREQ_START + FREQ_SKIP * index; }

constexpr unsigned long BROADCAST_TIME{20};

constexpr unsigned long WINDOW_LENGTH{5};

#else

#include "SoftwareSerialWithHalfDuplex.h"

constexpr uint8_t SERIAL_RX_PIN{10};
constexpr uint8_t SERIAL_TX_PIN{11};
constexpr uint8_t SERIAL_RATE{4800};

SoftwareSerialWithHalfDuplex mySerial(SERIAL_RX_PIN, SERIAL_TX_PIN, false, false);

const uint8_t WIRE_INDEXES[NUM_POLES] = {42 ,25 ,37 ,91 ,112,
                                         107,56 ,100, 62,89 ,
                                         123, 29, 53,71 ,45,
                                         14 , 77,122,86 ,22 };

#endif

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

unsigned int TxPeriod() {return (100 + ((5 * MY_INDEX) % 50));}

constexpr unsigned int CAP_SENSE_PERIOD {100};

enum DeviceState
{
  DS_WaitingReset,
  DS_WaitingCommand,
  DS_StateWritten,
};
volatile DeviceState state = DS_WaitingReset;

// scratchpad, with the CRC byte at the end
volatile byte scratchpad[9];

// This is the ROM the arduino will respond to, make sure it doesn't conflict with another device

#define CONNECT_READ_STATE 0xBE

auto hub = OneWireHub(pin_onewire);
OneWirePole* pole = nullptr;

int getbit(int pin, int bit){if (digitalRead(pin) == LOW) return 1 << bit ; else return 0;}

void setup()
{

  pinMode(3, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  delay(1);
  MY_INDEX = getbit(3,0) | getbit(A0,1) | getbit(A1,2) | getbit(A2,3) | getbit(A3,4);

  pinMode(3, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

#if ENABLE_PWD
SIGNAL_FREQ = signal_freq_for(MY_INDEX);
#endif 
pole = new  OneWirePole(MY_INDEX);

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

  hub.attach(*pole);

#ifdef USE_PWM
  FreqMeasure.begin();
#else
  mySerial.begin(SERIAL_RATE);
  delay(500);
#endif
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
#ifdef USE_PWM

bool checkConnect()
{
  static unsigned int lineindex = 0;
  static int count = 0;
  static unsigned long sum = 0;

  if (!FreqMeasure.available())
  {
    return false;
  }
  unsigned long now = millis();

  if (now > (lastread + READTIMEOUT))
  {
    count = 0;
    sum = 0;
  }

  while (FreqMeasure.available())
  {
    unsigned long reading = FreqMeasure.read();
    unsigned long freq = F_CPU/reading;

    if (freq < (FREQ_START - FREQ_SKIP)) {
      continue;
    }
    if (freq > (FREQ_END + FREQ_SKIP)) {
      continue;
    }
//Serial.println(freq);

    count++;
    sum += reading;

    if (count == WINDOW_LENGTH)
    {

      int poleindex = count_to_pole_index(sum / count);
      if (poleindex >= 0)
      {
            Serial.print(lineindex++);
             Serial.print(" detect ");
             Serial.println(poleindex);
             Serial.println(F_CPU / (sum / count));
        poletimes[poleindex] = now + TOUCH_TIMEOUT;
      }

      count = 0;
      sum = 0;
    }
  }

  lastread = now;
  return true;
}

int count_to_pole_index(unsigned long count)
{
  unsigned long freq = F_CPU / count;

  for (int i = 0; i < NUM_POLES; ++i)
  {
    unsigned long curfreq = signal_freq_for(i);
    unsigned long minfreq = curfreq - FREQ_SENSE;
    unsigned long maxfreq = curfreq + FREQ_SENSE;

    if ((minfreq < freq) && (freq <= maxfreq))
    {
      return i;
    }
  }
  return -1;
}

#else

bool checkConnect()
{
  static uint8_t windowindex = 0;
  static uint8_t lastreceived = 0xff;

  static unsigned int lineindex = 0;
  unsigned long now = millis();
  bool gotSomething = false;
  Serial.println("checking availability");
  while (mySerial.available() > 0)
  {
    byte readbyte = mySerial.read();

  Serial.print("we got some! availability");
  Serial.println((int)readbyte);
    if ((readbyte == 0xFF) || (readbyte == 0) || (readbyte == WIRE_INDEXES[MY_INDEX])  || (readbyte == ~WIRE_INDEXES[MY_INDEX]))
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
    int poleindex = - 1;
    for (int i = 0; i < COUNT_OF(WIRE_INDEXES); i++) {
      if ((readbyte == WIRE_INDEXES[i]) || (readbyte == ~WIRE_INDEXES[i])) {
        poleindex = i;
        break;
      }
    }

    if (poleindex < 0 )
    {
      return false;
    }
    if (poleindex >= NUM_POLES)
    {
      // impossible..
      return false;
    }

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
#endif

void loop()
{

  static uint8_t windowindex = 0;
  static unsigned int lineindex = 0;
  static unsigned long silencedeadline = 0;
  static unsigned long stateupdate_deadline = 0;

  hub.poll();

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
  }

  //     Serial.println(" got nothing");

  if (now < silencedeadline)
  {
    return;
  }

  if (now > capdeadline)
  {

    capdeadline = now + CAP_SENSE_PERIOD;

//    FreqMeasure.end();
mySerial.end();
//    pinMode(freqmeasure_pin, INPUT);
//    pinMode(signal_pin, INPUT);

    if (touchdetect())
    {
Serial.println("i'm touched");
      poletimes[MY_INDEX] = now + TOUCH_TIMEOUT;
    }
    pinMode(CAP_TX, INPUT);
    pinMode(CAP_RX, INPUT);
//    FreqMeasure.begin();
mySerial.begin(SERIAL_RATE);
  }

  if (now > txdeadline)
  {

    txdeadline = now + TxPeriod();
//    FreqMeasure.end();
mySerial.end();
//    pinMode(freqmeasure_pin, INPUT);
//    pinMode(signal_pin, INPUT);
    send_myself();
//    pinMode(signal_pin, INPUT);
    //FreqMeasure.begin();
    mySerial.begin(SERIAL_RATE);
  }
}

#ifdef USE_PWM

void send_myself()
{
  // start pwm on pin 8 9 or

  unsigned long micro = 1000000ul / SIGNAL_FREQ;
  Timer1.initialize(micro);
  Timer1.pwm(signal_pin, 512);
  // wait for time
  unsigned long deadine = millis() + BROADCAST_TIME;
  while (millis() < deadine)
  {
    hub.poll();
  }

  Timer1.disablePwm(signal_pin);
}
#else

void send_myself()
{

  uint8_t id = WIRE_INDEXES[MY_INDEX];
  uint8_t notid = ~id;
  for (int i = 0; i < 10; i++)
  {
    mySerial.write(&id, 1);
    mySerial.write(&notid, 1);
  }
}

#endif

void copyState()
{
  unsigned long now = millis();
  uint8_t currentstate[8] = {'\0'};

  bool istouch = false;
  bool isconnect = false;

  for (int i = 0; i < COUNT_OF(poletimes); i++)
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

  digitalWrite(TOUCH_LED_INDEX, istouch ? HIGH : LOW);
  digitalWrite(CONNECT_LED_INDEX, isconnect ? HIGH : LOW);

  pole->copy_scrachpad(currentstate, COUNT_OF(currentstate));
}