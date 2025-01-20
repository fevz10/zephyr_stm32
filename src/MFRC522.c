#include <stdio.h>
#include "MFRC522.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#define GPIO_CS_NODE    DT_ALIAS(cs)
#define SPI1_NODE       DT_CHOSEN(zephyr_spi)

static inline void Write_MFRC522(uint8_t addr, uint8_t val);
static inline uint8_t Read_MFRC522(uint8_t addr);
static inline uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen);
static inline void SetBitMask(uint8_t reg, uint8_t mask);
static inline void ClearBitMask(uint8_t reg, uint8_t mask);
static inline void CalculateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData);
static inline void AntennaOn(void);

const struct device *spi_dev;
struct gpio_dt_spec gpio_cs = GPIO_DT_SPEC_GET(GPIO_CS_NODE, gpios);

struct spi_config spi_cfg;
struct spi_cs_control chip;

struct spi_buf tx_buf;
struct spi_buf rx_buf;


static inline void Write_MFRC522(uint8_t addr, uint8_t val)
{
    uint8_t command = (addr << 1) & 0x7E;
    struct spi_buf_set tx_bufs;
    struct spi_buf txb[2];

    txb[0].buf = &command;
    txb[0].len = 1;
    txb[1].buf = &val;
    txb[1].len = 1;

    tx_bufs.buffers = txb;
    tx_bufs.count = 2;

    int ret = spi_write(spi_dev, &spi_cfg, &tx_bufs);
    if ( ret != 0 )
    {
        printk("SPI write failed with error : %d\n", ret);
    }

    //gpio_pin_set_dt(&gpio_cs, 0);
    /*
	RC522_SPI_Transfer((addr<<1)&0x7E);
	RC522_SPI_Transfer(val);
    */
    //gpio_pin_set_dt(&gpio_cs, 1);
}

static inline uint8_t Read_MFRC522(uint8_t addr)
{
    uint8_t command = ((addr << 1) & 0x7E) | 0x80; 
    uint8_t tx_data[2] = { command, 0x00 };
    uint8_t rx_data[2] = {0};

    tx_buf.buf = tx_data;
    tx_buf.len = 2;
    rx_buf.buf = rx_data;
    rx_buf.len = 2;

    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };

    if ( spi_transceive(spi_dev, &spi_cfg, &tx, &rx) != 0 )
    {
        printk("Error reading register\n");
        return 0;
    }

    return rx_data[1];
}

static inline uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEN = 0x00;
    uint8_t waitIRQ = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch ( command )
    {
        case PCD_AUTHENT:
        {
            irqEN = 0x12;
            waitIRQ = 0x10;
            break;
        }
        case PCD_TRANSCEIVE:
        {
            irqEN = 0x77;
            waitIRQ = 0x30;
            break;
        }
        default:
            break;
    }

    Write_MFRC522(CommIEnReg, irqEN|0x80);	// Interrupt request
    ClearBitMask(CommIrqReg, 0x80);			// Clear all interrupt request bit
    SetBitMask(FIFOLevelReg, 0x80);			// FlushBuffer=1, FIFO Initialization

	Write_MFRC522(CommandReg, PCD_IDLE);	// NO action; Cancel the current command

    // Writing data to the FIFO
    for ( i=0; i<sendLen; i++ )
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    // Execute the command
    Write_MFRC522(CommandReg, command);
    if ( command == PCD_TRANSCEIVE )
    {
        SetBitMask(BitFramingReg, 0x80);
    }

    i = 2000;
    do
    {
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ( (i!=0) && !(n&0x01) && !(n&waitIRQ) );

    ClearBitMask(BitFramingReg, 0x80);

    if ( i != 0 )
    {
        if ( !(Read_MFRC522(ErrorReg) & 0x01) )
        {
            status = MI_OK;
            if ( n & irqEN & 0x01 )
            {
                status = MI_NOTAGERR;
            }
            if ( command == PCD_TRANSCEIVE )
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if ( lastBits )
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }
                if ( n == 0 )
                {
                    n = 1;
                }
                if ( n > MAX_LEN )
                {
                    n = MAX_LEN;
                }

                // Reading the received data in FIFO
                for ( i = 0; i < n; i++ )
                {
                    backData[i] = Read_MFRC522(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }
    return status;
}

static inline void SetBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);
}

static inline void ClearBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}

static inline void CalculateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    uint8_t i, n;

    ClearBitMask(DivIrqReg, 0x04);
    SetBitMask(FIFOLevelReg, 0x80);

    // Writing data to the FIFO
    for ( i = 0; i < len; i++ )
    {
        Write_MFRC522(FIFODataReg, *(pIndata+i));
    }
    Write_MFRC522(CommandReg, PCD_CALCCRC);

    // Wait CRC calculation is complete
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ( (i != 0) && !(n & 0x04) );

    // Read CRC calculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegH);
}

static inline void AntennaOn(void)
{
	Read_MFRC522(TxControlReg);
	SetBitMask(TxControlReg, 0x03);
}



int SPI_Init(void)
{
    int ret;

    spi_dev = DEVICE_DT_GET(SPI1_NODE);
    if (!spi_dev)
    {
        printk("Failed to get SPI device...\n");
        return -1;
    }

    if ( !device_is_ready(gpio_cs.port) )
    {
        printk("GPIO CS device not ready\n");
    }
    ret = gpio_pin_configure_dt(&gpio_cs, GPIO_OUTPUT_ACTIVE);
    if ( ret != 0 )
    {
        printk("Failed to configure GPIO CS: %d", ret);
        return -1;
    }

    spi_cfg.frequency = 9000000;
    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA;
    spi_cfg.slave = 0;

    chip.gpio.port = gpio_cs.port;
    chip.gpio.pin = gpio_cs.pin;
    chip.gpio.dt_flags = GPIO_ACTIVE_LOW;
    chip.delay = 2;

    spi_cfg.cs = chip;
    printk("SPI Initialized.\n");
    return 0;
}

void MFRC522_Init(void)
{
    MFRC522_Reset();

	Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	Write_MFRC522(TReloadRegL, 30);
	Write_MFRC522(TReloadRegH, 0);
	Write_MFRC522(TxAutoReg, 0x40);		// force 100% ASK modulation
	Write_MFRC522(ModeReg, 0x3D);		// CRC Initial value 0x6363

    AntennaOn();
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *tagType)
{
    uint8_t status;
    uint16_t backBits;

    Write_MFRC522(BitFramingReg, 0x07);

    tagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
    
    if ( ( status != MI_OK ) || (backBits != 0x10) )
    {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_Anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint16_t unLen;

    Write_MFRC522(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if ( status == MI_OK )
    {
        // Check card serial number
        for ( i = 0; i < 4; i++ )
        {
            serNumCheck ^= serNum[i];
        }
        if ( serNumCheck != serNum[i] )
        {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t MFRC522_SelectTag(uint8_t *serNum)
{
    uint8_t i;
    uint8_t status;
    uint8_t size;
    uint16_t recvBits;
    uint8_t buffer[9];

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    
    for ( i = 0; i < 5; i++ )
    {
        buffer[i+2] = *(serNum+i);
    }

    CalculateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ( (status == MI_OK) && (recvBits == 0x18) )
    {
        size = buffer[0];
    }
    else 
    {
        size = 0;
    }

    return size;
}

uint8_t MFRC522_Auth(uint8_t authMode, uint8_t blockAddr, uint8_t *sectorKey, uint8_t *serNum)
{
    uint8_t status;
    uint16_t recvBits;
    uint8_t i;
    uint8_t buff[12];

    buff[0] = authMode;
    buff[1] = blockAddr;
    
    for ( i = 0; i < 6; i++ )
    {
        buff[i+2] = *(sectorKey+i);
    }
    for ( i = 0; i < 4; i++ )
    {
        buff[i+8] = *(serNum+i);
    }

    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ( (status != MI_OK) || ( !(Read_MFRC522(Status2Reg) & 0x08 ) ) )
    {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t *writeData)
{
    uint8_t status;
    uint16_t recvBits; 
    uint8_t i;
    uint8_t buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    
    CalculateCRC(buff, 2, &buff[2]);
    
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
    if ( (status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A ) )
    {
        status = MI_ERR;
    }

    if ( status == MI_OK )
    {
        for ( i = 0; i < 16; i++ )
        {
            buff[i] = *(writeData+i);
        }
        CalculateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

        if ( (status != MI_OK) || (recvBits != 4) || ( (buff[0] & 0x0F) != 0x0A ) )
        {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t *recvData)
{
    uint8_t status;
    uint16_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalculateCRC(recvData, 2, &recvData[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
    if ( (status != MI_OK) || (unLen != 0x90) )
    {
        status = MI_ERR;
    }

    return status;
}

void MFRC522_Halt(void)
{
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;

    CalculateCRC(buff, 2, &buff[2]);
    MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}