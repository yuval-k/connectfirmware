  
  #include "LowLevel.h"
  #include "OneWireSlave.h"
  
  #include "SoftwareSerialWithHalfDuplex.h"
  
  #define MY_INDEX   1

  #define LED_INDEX 13
  
  #define NUM_POLES 20
  #define TIMEOUT  500
  
  #define TX_PERIOD 60 + 40*MY_INDEX
  
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
  const byte owROM[7] = { 0x28, MY_INDEX, 0x00, 0x00, 0x00, 0x00, 0x02 };
  
  const byte CONNECT_READ_STATE = 0xBE;
  
  
  SoftwareSerialWithHalfDuplex mySerial(10, 11, false, false);
  
  void setup() {
    pinMode(LED_INDEX, OUTPUT);
    // put your setup code here, to run once:
    // try to read 4 bytes from the serial.
    // if all 4 bytes are the same, mark it is being connect
  
    // if 4 bytes are different (or other error) ???
  
    // if after timeout, write our id to the serial for X amount of time.
  
    // sleep random amunt
  
    // write our id to the serial
    mySerial.begin(38400);
  
    Serial.begin(115200);
    
    OWSlave.setReceiveCallback(&owReceive);
    OWSlave.begin(owROM, oneWireData.getPinNumber());
  }
  
  uint8_t window[4] = {0xff,0xff,0xff,0xff};
  uint8_t myid[] = {MY_INDEX,MY_INDEX,MY_INDEX,MY_INDEX,MY_INDEX};
  uint8_t index = 0;
  
  unsigned long poletimes[NUM_POLES] = {0};
  
  bool check() {
    if (window[0] != window[1]) return false;
    if (window[1] != window[2]) return false;
    if (window[2] != window[3]) return false;
    if (window[0] >= NUM_POLES) return false;
  
    return true;
  }
  void  clear_window() {
    window[0] = 0xff;
    window[1] = 0xff;
    window[2] = 0xff;
    window[3] = 0xff;

  }
  
  bool touchdetect()  {
    return false;
  // TODO
  }


unsigned long txdeadline = 0;
unsigned long lastread = 0;
#define READTIMEOUT 3
  
  void loop() {
    static unsigned int lineindex = 0;
  
    bool gotSomething = false;
    while (mySerial.available() > 0) {
      byte readbyte = mySerial.read();
      if (readbyte !=0) {
       gotSomething = true;
       unsigned long nowread = millis();

       if (nowread > (lastread + READTIMEOUT)) {
        clear_window();
        index = 0;
       }
         lastread = nowread;
        window[index] = readbyte;
        if(check()) {
          Serial.print(lineindex++);
          Serial.print(" detect");
          Serial.println(window[0]);  
          poletimes[window[0]] = nowread + TIMEOUT;
          
          copyState();
        } 
        index += 1;
        index %= (sizeof(window) / sizeof(window[0]));
      }
    }

    digitalWrite(LED_INDEX, gotSomething);

    if (gotSomething) {
      return;
    } 
    unsigned long now = millis();

 // TODO should we check for touch anyway?!
    if (now < txdeadline) {
      return;
    }
    txdeadline = now + TX_PERIOD;
    
    for (int i=0; i<2; i++) {
      mySerial.write(myid, sizeof(myid)/sizeof(myid[0]));
    }
  
    if (touchdetect()) {
      poletimes[MY_INDEX] = millis() + TIMEOUT;
      
      copyState();
    }
  
  }
  
  void copyState() {
  
    unsigned long now = millis();
    uint8_t currentstate[9] = {'\0'};
    for (int i = 0; i < NUM_POLES; i++){
      if (poletimes[i] < now) {
        currentstate[i>>3] |= (1 << (i & 0x7)); 
      }
    }
  
    currentstate[8] = OWSlave.crc8(currentstate, 8);
  
    cli();
    memcpy((void*)scratchpad, currentstate, 9);
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
          OWSlave.beginWrite((const byte*)scratchpad, 9, 0);
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
