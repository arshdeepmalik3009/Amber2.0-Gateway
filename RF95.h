#ifndef RF95_H
#define RF95_H

#include "LoRa.h"

#define RH_RF95_FIFO_SIZE 255
#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE
#define RH_RF95_HEADER_LEN 4

#ifndef RH_RF95_MAX_MESSAGE_LEN
 #define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)
#endif

class RF95: public LoRaClass
{
    public:
        /**
         * initialize the module
         */
        bool avail(void);
        bool recv(uint8_t *buf, uint8_t *len);
        bool send(const uint8_t *data, uint8_t len);
        void waitPacketSent();

};
#endif //RF95_H

