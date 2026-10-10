//
// Created by Andres on 06/28/26.
//

#ifndef WAVY_WAVY_H
#define WAVY_WAVY_H

#include "main.h"

#define DEFAULT_STACK_SIZE       512
/// Used to force placement of variables/functions/whatever into specific places
/// in specific places of ram
#define WAVYD1                   __attribute__((section(".d1ram")))
#define WAVYFD1                  __attribute__((section(".d1ramfunc")))
#define WAVYD3                   __attribute__((section(".d3ram")))
#define WAVYDTCM                 __attribute__((section(".dtcmram")))

// Maybe this isnt the right way
#define ADXL_BUFF_SIZE_UNALIGNED 2250
#define ADXL_BUFF_SIZE           2272
#define ADXL_CHUNK_SIZE          225

#define GPSRXSIZ                 1024

#define HOUR_PARSED              0b00000001
#define MINUTE_PARSED            0b00000010
#define SECONDS_PARSED           0b00000100
#define DAY_PARSED               0b00001000
#define MONTH_PARSED             0b00010000
#define YEAR_PARSED              0b00100000
#define DATE_OBTAINED            0b01000000
#define TIME_OBTAINED            0b10000000
#define PARSE_TIME               0x01
#define PARSE_DATE               0x09

#define PPS_TICK_THR_FLAG        0x0001U
#define GPS_RX_THR_FLAG          0x0002U
#define NMEA_PROC_THR_FLAG       0x0004U
#define ADXL_DATA_THR_FLAG       0x0008U
#define WAVY_INIT_THR_FLAG       0x0010U
#define SDWRITE_THR_FLAG         0x0020U
#define SDINIT_THR_FLAG          0x0040U
#define SDPREPW_THR_FLAG         0x0080U

struct wavyGPS
{
    char            format[6];
    char            date_str[7];
    char            time_str[7];
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
};

struct WavyPack
{
    // uint8_t gps[82];
    // FIXME: Need to check whats the current frequency of the adxl355
    uint8_t xyz[ADXL_BUFF_SIZE_UNALIGNED];
};

#endif // WAVY_WAVY_H
