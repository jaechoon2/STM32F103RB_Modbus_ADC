#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_MAX_LEN  256
#define MODBUS_SLAVE_ID 0x01

uint16_t modbus_crc16(uint8_t *buf, uint16_t len);
bool modbus_parse_request(uint8_t *rx_buf, uint16_t rx_len, uint8_t *tx_buf, uint16_t *tx_len);
uint16_t modbus_build_request(uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_count, uint8_t *tx_buf);

#endif // MODBUS_H
