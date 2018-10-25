#ifndef __ADS1256_H__
#define __ADS1256_H__

typedef enum
{
    PGA_GAIN1	= 0, // Input voltage range: +- 5 V
    PGA_GAIN2	= 1, // Input voltage range: +- 2.5 V
    PGA_GAIN4	= 2, // Input voltage range: +- 1.25 V
    PGA_GAIN8	= 3, // Input voltage range: +- 0.625 V
    PGA_GAIN16	= 4, // Input voltage range: +- 0.3125 V
    PGA_GAIN32	= 5, // Input voltage range: +- 0.15625 V
    PGA_GAIN64	= 6  // Input voltage range: +- 0.078125 V
} EM_PGA_GAIN;

typedef enum
{
    DRATE_30000 = 0xF0,
    DRATE_15000 = 0xE0,
    DRATE_7500  = 0xD0,
    DRATE_3750  = 0xC0,
    DRATE_2000  = 0xB0,
    DRATE_1000  = 0xA1,
    DRATE_500   = 0x92,
    DRATE_100   = 0x82,
    DRATE_60    = 0x72,
    DRATE_50    = 0x63,
    DRATE_30    = 0x53,
    DRATE_25    = 0x43,
    DRATE_15    = 0x33,
    DRATE_10    = 0x20,
    DRATE_5     = 0x13,
    DRATE_2d5   = 0x03
} EM_DRATE;


typedef enum
{
    REG_STATUS = 0,	 // Register adress: 00h, Reset value: x1H
    REG_MUX    = 1,  // Register adress: 01h, Reset value: 01H
    REG_ADCON  = 2,  // Register adress: 02h, Reset value: 20H
    REG_DRATE  = 3,  // Register adress: 03h, Reset value: F0H
    REG_IO     = 4,  // Register adress: 04h, Reset value: E0H
    REG_OFC0   = 5,  // Register adress: 05h, Reset value: xxH
    REG_OFC1   = 6,  // Register adress: 06h, Reset value: xxH
    REG_OFC2   = 7,  // Register adress: 07h, Reset value: xxH
    REG_FSC0   = 8,  // Register adress: 08h, Reset value: xxH
    REG_FSC1   = 9,  // Register adress: 09h, Reset value: xxH
    REG_FSC2   = 10, // Register adress: 0Ah, Reset value: xxH
} EM_REG;


typedef enum
{
    CMD_WAKEUP   = 0x00, // Completes SYNC and Exits Standby Mode
    CMD_RDATA    = 0x01, // Read Data
    CMD_RDATAC   = 0x03, // Read Data Continuously
    CMD_SDATAC   = 0x0F, // Stop Read Data Continuously
    CMD_RREG     = 0x10, // Read from REG - 1st command byte: 0001rrrr
    //					2nd command byte: 0000nnnn
    CMD_WREG     = 0x50, // Write to REG  - 1st command byte: 0001rrrr
    //					2nd command byte: 0000nnnn
    // r = starting reg address, n = number of reg addresses
    CMD_SELFCAL  = 0xF0, // Offset and Gain Self-Calibration
    CMD_SELFOCAL = 0xF1, // Offset Self-Calibration
    CMD_SELFGCAL = 0xF2, // Gain Self-Calibration
    CMD_SYSOCAL  = 0xF3, // System Offset Calibration
    CMD_SYSGCAL  = 0xF4, // System Gain Calibration
    CMD_SYNC     = 0xFC, // Synchronize the A/D Conversion
    CMD_STANDBY  = 0xFD, // Begin Standby Mode
    CMD_RESET    = 0xFE, // Reset to Power-Up Values
} EM_CMD;


typedef enum
{
    AIN0   = 0, //Binary value: 0000 0000
    AIN1   = 1, //Binary value: 0000 0001
    AIN2   = 2, //Binary value: 0000 0010
    AIN3   = 3, //Binary value: 0000 0011
    AIN4   = 4, //Binary value: 0000 0100
    AIN5   = 5, //Binary value: 0000 0101
    AIN6   = 6, //Binary value: 0000 0110
    AIN7   = 7, //Binary value: 0000 0111
    AINCOM = 8  //Binary value: 0000 1000
} EM_AIN;



#define SPI_WRITE_THEN_READ 0xFAFA0000
#define ADC_START_DATA_STREAM 0xFAF10000
#define ADC_STOP_DATA_STREAM 0xFAF20000
#define ADC_RESET_HW 0xFAF30000
#define ADC_SYNC_HW 0xFAF50000
#define ADC_WAIT_READY 0xFAF60000

#define ADC_READ_FIFO 0xFAF90000

#define ADC_FIFO_SIZE 1000

typedef enum
{
    D_X,
    D_Y,
    D_Z,
    D_NOTANY
} EM_DIRECT;


enum ADS1256_CMD
{
    ADS1256_CFG_SPI,
    ADS1256_CFG_AD,
    //ADS1256_NULL,
    ADS1256_START,
    ADS1256_STOP,
    ADS1256_CALIBRATION,
    ADS1256_READ_CALIB,
    ADS1256_READDATA,
    ADS1256_RESET
};

struct ads1256_spi_config {
    unsigned int speed_hz;
    unsigned int bpw;
    unsigned int mode;
    u8 cs;
};

struct ads1256_ad_config {
    unsigned int sample_rate;
};

#define ADD_POINT_EN  1

/*valid说明:
bit0:当前数据是否有效
bit1:当前数据是否曾经校正过
bit2:当前数据是否序号错误
bit3:这个数据前一个数据是否不连续(丢包)
bit4:是否是秒边缘数据
bi5->bit7:保留
bit8->bit15:保留
bit16->bit23:当前同步状态，对应SYNC_STATE
*/
typedef union {
    struct {
        uint32_t data_valid_:1;
        uint32_t adjusted_:1;
        uint32_t index_err_:1;
        uint32_t lost_err_:1;
        uint32_t data_count_err_:1;
        uint32_t is_edge_:1;
        uint32_t res1_:10;
        uint32_t sync_state_:8;
        uint32_t res2_:8;
    } sinfo_;
    uint32_t linfo_;
} adc_valid_info;

typedef struct
{
    int32_t data[3];
    int32_t gps_sig;
    uint64_t timestamp;
    uint64_t timestamp_sys;
#if ADD_POINT_EN
    adc_valid_info info;
#endif
} AdcData;


#pragma pack(push,1)
typedef struct
{
    uint8_t  flagx;
    uint8_t  datax[3];
    uint8_t  flagy;
    uint8_t  datay[3];
    uint8_t  flagz;
    uint8_t  dataz[3];
} AdcOrgData;
#pragma pack(pop)

#endif
