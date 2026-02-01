#include "protocol.h"
#include "eeprom.h"
#include "main.h"

extern EEPROM_t eeprom;
extern void UART5_Transmit(uint8_t *data, uint16_t size);

void Protocol_SendResponse(uint8_t op, uint8_t *data, uint8_t data_len) {
  uint8_t frame[RX_BUFFER_SIZE];
  uint8_t idx = 0;
  frame[0] = 0xAA;
  frame[1] = 0x55;
  // LEN placeholder at index 2, filled after
  frame[3] = op | 0x40; // set response bit
  idx = 4;
  for (uint8_t i = 0; i < data_len; i++) {
    frame[idx++] = data[i];
  }
  frame[idx++] = 0x55;
  frame[idx++] = 0xAA;
  frame[2] = idx; // LEN = total packet length
  UART5_Transmit(frame, idx);
}

void Protocol_ProcessPacket(uint8_t *buf, uint8_t len) {
  // Packet: [0xAA][0x55][LEN][OP][...data...][0x55][0xAA]
  if (len < 6)
    return; // minimum: header(2) + LEN(1) + OP(1) + footer(2)

  uint8_t op = buf[3];
  uint8_t *payload = &buf[4];
  uint8_t payload_len =
      len - 6; // exclude header(2) + LEN(1) + OP(1) + footer(2)

  uint8_t is_write = (op >> 7) & 1;
  uint8_t number = op & 0x3F;

  if (number == 11) {
    // Record count
    if (is_write) {
      if (payload_len >= 1) {
        EEPROM_Write(&eeprom, EEPROM_COUNT_ADDR, &payload[0], 1, 100);
        Protocol_SendResponse(op, NULL, 0);
      }
    } else {
      uint8_t count = 0;
      EEPROM_Read(&eeprom, EEPROM_COUNT_ADDR, &count, 1, 100);
      Protocol_SendResponse(op, &count, 1);
    }
  } else if (number >= 12) {
    uint8_t record_idx = number - 12;
    uint16_t addr =
        EEPROM_RECORDS_BASE + (uint16_t)record_idx * EEPROM_RECORD_SIZE;
    if (is_write) {
      uint8_t write_len = payload_len;
      if (write_len > EEPROM_RECORD_SIZE)
        write_len = EEPROM_RECORD_SIZE;
      EEPROM_Write(&eeprom, addr, payload, write_len, 100);
      Protocol_SendResponse(op, NULL, 0);
    } else {
      uint8_t record_data[EEPROM_RECORD_SIZE];
      EEPROM_Read(&eeprom, addr, record_data, EEPROM_RECORD_SIZE, 100);
      Protocol_SendResponse(op, record_data, EEPROM_RECORD_SIZE);
    }
  }
}
