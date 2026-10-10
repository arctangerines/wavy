//
// Created by Andres on 06/28/26.
//

#include "axl.h"

#include "cmsis_os2.h"

// FIXME: Add error handling for transmission errors
void
adxl355_read(SPI_HandleTypeDef* spi,
             const uint8_t      reg,
             uint8_t*           data,
             uint16_t           size)
{
    uint8_t reg_op[1] = {(reg << 1) | ADXL_READ};
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, reg_op, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_read_data(SPI_HandleTypeDef* spi,
                  uint8_t*           buffer)
{
    uint8_t regs[9] = {
        (ADXL_REG_XDATA3 << 1) | ADXL_READ, (ADXL_REG_XDATA2 << 1) | ADXL_READ,
        (ADXL_REG_XDATA1 << 1) | ADXL_READ, (ADXL_REG_YDATA3 << 1) | ADXL_READ,
        (ADXL_REG_YDATA2 << 1) | ADXL_READ, (ADXL_REG_YDATA1 << 1) | ADXL_READ,
        (ADXL_REG_ZDATA3 << 1) | ADXL_READ, (ADXL_REG_ZDATA2 << 1) | ADXL_READ,
        (ADXL_REG_ZDATA1 << 1) | ADXL_READ,
    };
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, regs, 1, HAL_MAX_DELAY);
    // HAL_SPI_Receive(spi, buffer, 9, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, buffer, 9, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_write(SPI_HandleTypeDef* spi,
              const uint8_t      reg,
              const uint8_t      data,
              uint16_t           size)
{
    uint8_t reg_opd[2];
    reg_opd[0] = (reg << 1) | ADXL_WRITE;
    reg_opd[1] = data;
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, reg_opd, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_fifo_read(SPI_HandleTypeDef* spi,
                  uint8_t*           xyz_data)
{
    uint8_t r = (ADXL_REG_FIFO_DATA << 1) | ADXL_READ;
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &r, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, xyz_data, XYZBUFSIZ, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_init(SPI_HandleTypeDef* spi)
{
    uint8_t device_id   = 0;
    uint8_t range_bits  = 0xFF;
    uint8_t power_bits  = 0xFF;
    uint8_t filter_bits = 0x0;
    adxl355_read(spi, ADXL_REG_DEVID, &device_id, 1);
    adxl355_read(spi, ADXL_REG_RANGE, &range_bits, 1);
    adxl355_read(spi, ADXL_REG_POWER_CTL, &power_bits, 1);
    adxl355_read(spi, ADXL_REG_FILTER_OPTS, &filter_bits, 1);
    if (device_id != 0xAD)
    {
        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
        for (int i = 0; i < 6; i++)
        {
            // HAL_Delay() because cmsis not enabled yet
            HAL_Delay(500);
            HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN);
        }
        Error_Handler();
    }
    if (filter_bits != ADXL_250HZ)
    {
        adxl355_write(spi, ADXL_REG_FILTER_OPTS, ADXL_250HZ, 1);
    }
    if (range_bits != 0x01)
    {
        adxl355_write(spi, ADXL_REG_RANGE, ADXL_RANGE_2G, 1);
    }
    // The last bit needs to be 0 to stop standby mode
    if ((power_bits & 0x01) != 0x00)
    {
        adxl355_write(spi, ADXL_REG_POWER_CTL, power_bits & 0xFE, 1);
    }
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
    for (int i = 0; i < 6; i++)
    {
        // HAL_Delay() because cmsis not enabled yet
        HAL_Delay(300);
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
    }
}
