#ifndef ONEWIRE_POLE_H
#define ONEWIRE_POLE_H

#include "utils.h"

#include "OneWireItem.h"

#define CONNECT_READ_STATE 0xBE

class OneWirePole : public OneWireItem
{
private:

    uint8_t scratchpad[4];

   void updateCRC();


public:
  inline void copy_scrachpad(const uint8_t* data, size_t len) {
      int sizetocopy = min(len, COUNT_OF(scratchpad)-1);
      memcpy((void *)scratchpad, data, sizetocopy);
      updateCRC();
  }

    OneWirePole(int index);
    void duty(OneWireHub * const hub);
};

#endif
