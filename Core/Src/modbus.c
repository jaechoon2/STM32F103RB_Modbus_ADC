#include "modbus.h"

uint16_t modbus_crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t modbus_build_request(uint8_t slave_id, uint8_t func_code, uint16_t reg_addr, uint16_t reg_count, uint8_t *tx_buf)
{
    tx_buf[0] = slave_id;
    tx_buf[1] = func_code;
    tx_buf[2] = reg_addr >> 8;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = reg_count >> 8;
    tx_buf[5] = reg_count & 0xFF;
    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = crc >> 8;
    return 8;
}

bool modbus_parse_request(uint8_t *rx_buf, uint16_t rx_len, uint8_t *tx_buf, uint16_t *tx_len)
{
    if (rx_len < 8) return false;

    uint16_t crc_received = (rx_buf[rx_len - 1] << 8) | rx_buf[rx_len - 2];
    uint16_t crc_calc = modbus_crc16(rx_buf, rx_len - 2);
    if (crc_received != crc_calc) return false;

    uint8_t slave_id = rx_buf[0];
    uint8_t func = rx_buf[1];
    if (slave_id != MODBUS_SLAVE_ID) return false;

    if (func == 0x03) {
        tx_buf[0] = slave_id;
        tx_buf[1] = func;
        tx_buf[2] = 2;
        tx_buf[3] = 0x12;
        tx_buf[4] = 0x34;
        uint16_t crc = modbus_crc16(tx_buf, 5);
        tx_buf[5] = crc & 0xFF;
        tx_buf[6] = crc >> 8;
        *tx_len = 7;
        return true;
    }

    return false;
}
