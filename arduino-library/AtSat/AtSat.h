#ifndef __ATSAT_H__
#define __ATSAT_H__

#include "Arduino.h"
#include <stdint.h>

static const int ATSAT_PACKET_MAX_DATA_LENGTH = 128;

// Structure for AXTEC packet
typedef struct
{
    uint16_t length;                                // Data length
    uint8_t raw[ATSAT_PACKET_MAX_DATA_LENGTH];      // Packet raw bytes
} atsat_encoded_packet_t;

class AtSat
{
private:
    static const uint8_t SEND_DATA_CMD = 0x0C;
    static const int MAX_DATA_LENGTH = 50;
    static const int ATSAT_PACKET_START_BYTE = 0x7E;
    static const int ATSAT_PACKET_ESCAPE_BYTE = 0x7D;

    uint8_t tempBuffer[MAX_DATA_LENGTH];

    bool _debug;

    bool buildPacket(atsat_encoded_packet_t *packet, uint8_t data[], size_t length);
    void sendPacket(atsat_encoded_packet_t *packet);
    bool sendSensor(uint8_t sensorId, uint8_t data[], size_t size);

public:

    AtSat(bool debugMode);

    void init(void);
    bool sendSensor(uint8_t sensorId, float data);
};

#endif