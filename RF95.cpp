#include "RF95.h"

bool RF95::avail() {
    return isRX_DoneSet();
}

bool RF95::recv(uint8_t *buf, uint8_t *len) {
    int i;

    int packetSize = parsePacket();
    if (packetSize) {
        // received a packet

        // read packet
        i = 0;
        while (available()) {
            buf[i++] = read();
        }
        *len = i;

        return true;
    }
    else
        return false;
}

bool RF95::send(const uint8_t *data, uint8_t len) {
    beginPacket(false);
    write(data, len);
    endPacket();
    return true;
}

void RF95::waitPacketSent() {
    //already implemented in endPacket
}

