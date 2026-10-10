//
// Created by Andres Iglesias on 06/28/26.
//

#ifndef WAVY_AXL_H
#define WAVY_AXL_H
#include "wavy.h"

/// The ADXL355 captures data in LSB/g.
/// To turn this into a properly usable unit we have to take some steps.
/// 1. We calculate the full range of values.
/// To do this we obtain the maximum LSB value: 2 ^ n-1, where n is
/// the amount of bits in the ADC, the ADXL355 has a 20-bit ADC, thus
/// 2^19 is our LSB.
/// Then we obtain our range of g, since we use ±2g,
/// our range of possible values is 2g since data provided by
/// ADXL355 is already in twos complement.
/// We do 2^19 LSB / 2g which gives 262,144 LSB/g.
/// 2. Convert the captured raw data (LSB) to g.
/// This step is done by dividing a raw value with the full range value.
/// e.g. -245000 LSB / 262144 LSB/g = -0.935 g.
/// Please note that the sign comes from the fact that the raw data
/// is in twos complement.
///
/// If you grab this full range value 262,144 LSB/g, which is our "sensitivity
/// value" which we will call w from now, and you do 1/w, you get the sensitivity
/// value found in the data sheet, but but but, now that we know where it comes
/// from, we are better off using the sensitivity value 3.9 𝝁g/LSB, which we will
/// call sm, the m for the manufacturer. data ⋅ sm is better than data / w for
/// efficiency. In fact, I would advice against doing these corrections at the MCU
/// level, and focus on doing them after retrieval or at the broker.

/// Status register bit flags
/// x, y and z axes measurements are ready and can be read.
#define ADXL_FLAG_DATA_RDY    0x01
/// The FIFO is full, this value can be set
#define ADXL_FLAG_FIFO_FULL   0x02
#define ADXL_FLAG_FIFO_OVR    0x04
#define ADXL_FLAG_ACTIVITY    0x08
/// ADXL read and write commands, these are prefixes in the messages sent
#define ADXL_WRITE            0x00
#define ADXL_READ             0x01
/// Contains the Analog Devices ID - RO - Returns 0xAD
#define ADXL_REG_DEVID        0x00
/// Status register, uses bit flags to describe different states - RO
#define ADXL_REG_STATUS       0x04
/// Number of data samples in the FIFO buffer - RO
#define ADXL_REG_FIFO_ENTRIES 0x05
/// X Data values ///
/// Bits[19:12]
#define ADXL_REG_XDATA3       0x08
/// Bits[11:4]
#define ADXL_REG_XDATA2       0x09
/// Bits[3:0] (the first 4 LSB are reserved)
#define ADXL_REG_XDATA1       0x0A
/// Y Data values ///
/// Bits[19:12]
#define ADXL_REG_YDATA3       0x0B
/// Bits[11:4]
#define ADXL_REG_YDATA2       0x0C
/// Bits[3:0] (the first 4 LSB are reserved)
#define ADXL_REG_YDATA1       0x0D
/// Z Data values ///
/// Bits[19:12]
#define ADXL_REG_ZDATA3       0x0E
/// Bits[11:4]
#define ADXL_REG_ZDATA2       0x0F
/// Bits[3:0] (the first 4 LSB are reserved)
#define ADXL_REG_ZDATA1       0x10
/// Data stored in the FIFO lives here - RO
#define ADXL_REG_FIFO_DATA    0x11
/// Filter setting register, sets parameters for the filters and frequency - RW
#define ADXL_REG_FILTER_OPTS  0x28
/// Stores the number of values the FIFO can hold - RW - Defaults to 0x60 (96)
#define ADXL_REG_FIFO_SAMPLES 0x29
/// Allows toggling different power modes of the ADXL355 - RW
#define ADXL_REG_POWER_CTL    0x2D
/// Used to change some settings on the 355, we will use it for the g range - RW
#define ADXL_REG_RANGE        0x2C

/// For the ADXL_REG_RANGE
/// Bits [1:0] are for the g range i.e. ±ng, where n is an integer and g is
/// the unit
#define ADXL_RANGE_2G         0x01
#define ADXL_RANGE_4G         0x02
#define ADXL_RANGE_8G         0x03

/// For the ADXL_REG_FILTER_OPTS
/// Bits [3:0] are for ODR and LPF corner
#define ADXL_1000HZ           0x02
#define ADXL_250HZ            0x04
#define ADXL_125HZ            0x05

/// Bits [6:4] are for the HPF corner
#define ADXL_NOHPF            0b0000

void
adxl355_read(SPI_HandleTypeDef* spi,
             const uint8_t      reg,
             uint8_t*           data,
             uint16_t           size);
void
adxl355_read_data(SPI_HandleTypeDef* spi,
                  uint8_t*           buffer);
void
adxl355_write(SPI_HandleTypeDef* spi,
              const uint8_t      reg,
              const uint8_t      data,
              uint16_t           size);
#define XYZBUFSIZ 90
void
adxl355_fifo_read(SPI_HandleTypeDef* spi,
                  uint8_t*           xyz_data);

void
adxl355_init(SPI_HandleTypeDef* spi);

#endif // WAVY_AXL_H
