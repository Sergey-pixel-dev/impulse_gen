#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define EEPROM_RESERVED_SIZE  32
#define EEPROM_COUNT_ADDR     32
#define EEPROM_RECORDS_BASE   33  // 0x0021
#define EEPROM_RECORD_SIZE    209

void Protocol_ProcessPacket(uint8_t *buf, uint8_t len);
void Protocol_SendResponse(uint8_t op, uint8_t *data, uint8_t data_len);

#endif // PROTOCOL_H
