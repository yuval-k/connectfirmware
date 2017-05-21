#include "OneWirePole.h"
#include "utils.h"
#include <OneWire.h>

OneWirePole::OneWirePole(int index) : OneWireItem(0x33, index,0,0,0,0,0), scratchpad{0}
{
   
}


void OneWirePole::updateCRC()
{
    scratchpad[COUNT_OF(scratchpad)-1] = crc8(scratchpad, COUNT_OF(scratchpad)-1);
}

void OneWirePole::duty(OneWireHub * const hub)
{
    uint8_t cmd;
    if (hub->recv(&cmd,1)) return;

    switch (cmd)
    {
        case CONNECT_READ_STATE: // READ SCRATCHPAD
            hub->send(scratchpad, 9);
            break;

        default:
            hub->raiseSlaveError(cmd);
    };
};

