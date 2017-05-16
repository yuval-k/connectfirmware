#include "OneWireHub.h"
#include "OneWirePole.h"

#include "utils.h"

#include "LowLevel.h"
#include "CapTouch.h"

constexpr int MY_INDEX{1};
constexpr int NUM_POLES{20};

#define USE_PWM

#ifdef USE_PWM

#include "FreqCount.h"
#include "TimerOne.h"

constexpr unsigned long FREQ_TIMEOUT{100};

constexpr unsigned long freqCFreqCount_pin{5};
constexpr unsigned long signal_pin{9};

constexpr unsigned long FREQ_START{1000};
constexpr unsigned long FREQ_END{2000};

constexpr unsigned long FREQ_SKIP{(FREQ_END - FREQ_START) / (NUM_POLES - 1)};
constexpr unsigned long FREQ_SENSE{FREQ_SKIP * 20 / 100}; // 20%

constexpr unsigned long SIGNAL_FREQ{FREQ_START + FREQ_SKIP * MY_INDEX};
constexpr unsigned long signal_freq_for(int index) { return FREQ_START + FREQ_SKIP * index; }

constexpr unsigned long BROADCAST_TIME{20};
constexpr unsigned long GATE_INTERVAL{5};

#else

#include "SoftwareSerialWithHalfDuplex.h"

SoftwareSerialWithHalfDuplex mySerial(10, 11, false, false);

#endif

constexpr uint8_t TOUCH_LED_INDEX{13};
constexpr uint8_t CONNECT_LED_INDEX{12};

constexpr uint8_t CAP_TX{4};
constexpr uint8_t CAP_RX{6};

constexpr uint8_t pin_onewire{7};

#define CAP_SAMPLES 15
#define CAP_THRESHOLD 100

#define SILENCE_AFTER_READ 20

#define STATE_UPDATE 50

CapTouch capTouch = CapTouch(CAP_TX, CAP_RX);

#define TOUCH_TIMEOUT 500

#define TX_PERIOD (50 + ((5 * MY_INDEX) % 50))

#define CAP_SENSE_PERIOD 100

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
auto pole = OneWirePole(MY_INDEX);
void setup()
{

  pinMode(TOUCH_LED_INDEX, OUTPUT);
  pinMode(CONNECT_LED_INDEX, OUTPUT);

  Serial.begin(115200);
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

  hub.attach(pole);

#ifdef USE_PWM
 FreqCount.begin(GATE_INTERVAL); 
#else
  mySerial.begin(38400);
  delay(500);
#endif
}

#define SEND_INDEX (MY_INDEX + 1)

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
#ifdef USE_PWM

bool checkConnect()
{
  static unsigned int lineindex = 0;
  static int count = 0;
  static unsigned long sum = 0;

  if (!FreqCount.available())
  {
    return false;
  }
  unsigned long now = millis();

  if (now > (lastread + READTIMEOUT))
  {
    count = 0;
    sum = 0;
  }

  while (FreqCount.available())
  {
    count++;
    unsigned long reading = FreqCount.read();
    if (reading <= 1 ) {
      return false;
    }
    sum += reading;
    Serial.print(reading);
    Serial.print(" ");
    Serial.println((reading)*1000/GATE_INTERVAL);

    if (count == 2)
    {

    Serial.println((sum / count)*1000/GATE_INTERVAL);
      int poleindex = count_to_pole_index(  sum / count);
      if (poleindex >= 0)
      {
        Serial.print(lineindex++);
        Serial.print(" detect");
        Serial.println(poleindex);
        Serial.println(sum / count);
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
  unsigned long freq = count*1000/GATE_INTERVAL;

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
  while (mySerial.available() > 0)
  {
    byte readbyte = mySerial.read();
    if ((readbyte == 0) || (readbyte == SEND_INDEX))
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
    
    FreqCount.end();
      pinMode(freqCFreqCount_pin, INPUT);
      pinMode(signal_pin, INPUT);

    if (touchdetect())
    {

      poletimes[MY_INDEX] = now + TOUCH_TIMEOUT;
    }
    pinMode(CAP_TX, INPUT);
    pinMode(CAP_RX, INPUT);
    FreqCount.begin(GATE_INTERVAL);
    
  }

  if (now > txdeadline)
  {
    
    txdeadline = now + TX_PERIOD;
    FreqCount.end();
    pinMode(freqCFreqCount_pin, INPUT);
    pinMode(signal_pin, INPUT);
    send_myself();
    pinMode(signal_pin, INPUT);    
    FreqCount.begin(GATE_INTERVAL);
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
  for (int i = 0; i < 2; i++)
  {
    mySerial.write(myid, COUNT_OF(myid));
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

  pole.copy_scrachpad(currentstate, COUNT_OF(currentstate));
}