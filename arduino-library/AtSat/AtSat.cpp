#include "AtSat.h"
#include "Arduino.h"
#include <stdint.h>
#include <string.h>

AtSat::AtSat(bool debugMode)
{
    _debug = debugMode;
}

void AtSat::init(void) {
    Serial.begin(9600);
}

bool AtSat::buildPacket(atsat_encoded_packet_t* packet, uint8_t data[], size_t length) {
    // Worst case, 1 start byte and 4 bytes for length when both are escaped
    if((length + 5) > ATSAT_PACKET_MAX_DATA_LENGTH)
    {
        return false;
    }
    unsigned int index = 0;

    // Add start byte
    packet->raw[index++] = ATSAT_PACKET_START_BYTE;

    // Add length bytes, MSB first
    uint8_t msb_l = (uint8_t)(length >> 8);
    uint8_t lsb_l = (uint8_t)length;
    // Length MSB byte
    if(msb_l == ATSAT_PACKET_START_BYTE || msb_l == ATSAT_PACKET_ESCAPE_BYTE)
    {
        packet->raw[index++] = ATSAT_PACKET_ESCAPE_BYTE;
        packet->raw[index++] = msb_l ^ 0x20;
    }
    else
    {
        packet->raw[index++] = msb_l;
    }
    // Length LSB byte
    if(lsb_l == ATSAT_PACKET_START_BYTE || lsb_l == ATSAT_PACKET_ESCAPE_BYTE)
    {
        packet->raw[index++] = ATSAT_PACKET_ESCAPE_BYTE;
        packet->raw[index++] = lsb_l ^ 0x20;
    }
    else
    {
        packet->raw[index++] = lsb_l;
    }

    // Add all the bytes and escape them if needed. Compute the checksum too.
    uint8_t checksum = 0;
    for(unsigned int n = 0; n < length; ++n)
    {
        // Check length
        if(index > ATSAT_PACKET_MAX_DATA_LENGTH)
        {
            // Exceeded max packet length
            return false;
        }

        // Need escaping?
        if(data[n] == ATSAT_PACKET_START_BYTE || data[n] == ATSAT_PACKET_ESCAPE_BYTE)
        {
            if(index + 1 > ATSAT_PACKET_MAX_DATA_LENGTH)
            {
                // Exceeded max packet length
                return false;
            }
            checksum += data[n];
            packet->raw[index++] = ATSAT_PACKET_ESCAPE_BYTE;
            packet->raw[index++] = data[n] ^ 0x20;
        }
        else
        {
            checksum += data[n];
            packet->raw[index++] = data[n];
        }
    }

    // Add the checksum
    if(index > ATSAT_PACKET_MAX_DATA_LENGTH)
    {
        // Exceeded max packet length
        return false;
    }
    packet->raw[index] = 0xFF - checksum;

    packet->length = index + 1;
    return true;
}

void AtSat::sendPacket(atsat_encoded_packet_t *packet) {
    Serial.write(packet->raw, packet->length);
}

bool AtSat::sendSensor(uint8_t sensorId, float data) {
    if(!_debug) {
        uint8_t bytes[4];
        uint8_t *ptr = (uint8_t *) &data;
        
        bytes[0] = *ptr++;
        bytes[1] = *ptr++;
        bytes[2] = *ptr++;
        bytes[3] = *ptr++;
        
        return sendSensor(sensorId, bytes, sizeof(bytes));
    }
    else {
        Serial.print("Envio de sensor '");
        Serial.print(sensorId);
        Serial.print("' con valor '");
        Serial.print(data);
        Serial.println("'");

        return true;
    }
}

bool AtSat::sendSensor(uint8_t sensorId, uint8_t data[], size_t size) {
    if(size >= MAX_DATA_LENGTH) {
        return false;
    }

    tempBuffer[0] = SEND_DATA_CMD;
    tempBuffer[1] = sensorId;
    memcpy(tempBuffer + 2, data, size);

    atsat_encoded_packet_t packet;
    if(buildPacket(&packet, data, size)) {
        sendPacket(&packet);
        return true;
    }
    
    return false;
}