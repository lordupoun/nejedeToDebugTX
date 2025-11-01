/*
 * SI4432 basic library
 *
 */
#include "SI4432.h"
#include "SI4432_private.h"
#include "SI4432_IO.h"
#include <stdio.h>
//#include "stm32f1xx_hal_spi.h"

const uint8_t address[]={0x1C,
		0x1D,
		0x1E,
		0x1F,
		0x20,
		0x21,
		0x22,
		0x23,
		0x24,
		0x25,
		0x2A,
		0x2C,
		0x2D,
		0x2E,
		0x30,
		0x32,
		0x33,
		0x34,
		0x35,
		0x36,
		0x37,
		0x38,
		0x39,
		0x3A,
		0x3B,
		0x3C,
		0x3D,
		0x3E,
		0x3F,
		0x40,
		0x41,
		0x42,
		0x43,
		0x44,
		0x45,
		0x46,
		0x58,
		0x69,
		0x6E,
		0x6F,
		0x70,
		0x71,
		0x72,
		0x75,
		0x76,
		0x77
};

const uint8_t byte[]={0x9A,
		0x44,
		0x0A,
		0x03,
		0x3C,
		0x02,
		0x22,
		0x22,
		0x07,
		0xFF,
		0x48,
		0x28,
		0x0C,
		0x28,
		0xAC,
		0x8C,
		0x02,
		0x0A,
		0x2A,
		0x2D,
		0xD4,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xC0,
		0x60,
		0x19,
		0x9A,
		0x0C,
		0x23,
		0x50,
		0x53,
		0x4B,
		0x00
};

uint8_t SI44_CalcFrequencyDeviationRegister(uint32_t deviation)
{
    return (uint8_t) (deviation/625);
}

void SI44_CalcFrequencyCarierRegisters(float freq, uint8_t * regs)
{
    uint32_t fb = 0;
    uint8_t hbsel = 0b01000000;

    uint32_t F = freq*1000;

    if (F < 480000)
    {
        fb = (F - 240000);
    } else {
        fb = (F - 480000)/2;
        hbsel |= 0b00100000;
    }
    regs[0] = hbsel | ((uint8_t)(fb/10000) & 0b00011111);
    uint16_t fc = 0;
    //printf("\n\n%i %i\n\n", F, (fb/10000)*10000);
    fc = (uint16_t) ((F/(1 + (F > 480000 ? 1 : 0) ) - 240000 - (fb/10000)*10000 )*100000/15625);
    //printf("\n\n%f\n\n", (freq/((1.0 + (freq > 480.0 ? 1.0 : 0) )) - 240.0 - fb*10 ));
    //printf("\n\n%i\n\n", fc);
    regs[1] = (uint8_t) ((fc >> 8) & 0xff);
    regs[2] = (uint8_t) (fc & 0xff);
}

void SI44_CalcDataRateRegisters(uint16_t baudrate, uint8_t * regs)
{
    //uint64_t val = 1;// = 2097152;
    //val <<= 21;
    uint64_t val =  (uint64_t)baudrate << 21;
    val /= 1000000;
    //printf("\n\n%i\n\n", val);
    //val = baudrate*2.097152;
    regs[0] = (uint8_t) ((val >> 8) & 0xff);
    regs[1] = (uint8_t) (val & 0xff);
}

void SI44_CalcPHRegisters(si44_ph_config config, uint8_t * regs)
{
    regs[0] = config.path | config.crc | config.crc_type;
    regs[1] = config.header | config.sync;
    regs[2] = config.preambule_length;
}

void SI44_CalcConfigRegisters(si44_config config, uint8_t * regs)
{
    regs[0] = 0b00000000 | config.encoding_options; //ToDo - lower bitrates need 0b00100000
    regs[1] = config.modulation_source | config.modulation_type;
}

uint8_t SI44_ReadStatus(void)
{
    uint8_t buf[1];
    SI44_Read(SI44_REG_STATUS, buf, 1);
    return buf[0];
}

void SI44_Reset(void)
{
    uint8_t reset = SI44_OPERATION_MODE_DEFAULT | 0b10000000;
    SI44_Write(SI44_REG_CTRL1, &reset, 1);
}

void SI44_Init(SPI_HandleTypeDef * hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    SI44_IO_Init(hspi, GPIOx, GPIO_Pin); //tady by se podle datasheetu mělo čekat na IRQ //ToDo:
    SI44_Reset();
}

void SI44_SetConfig(si44_config * conf)
{
    uint8_t buf[2];
    SI44_CalcConfigRegisters(*conf, buf);
    SI44_Write(SI44_REG_CONF1, &buf[0], 1);
    SI44_Write(SI44_REG_CONF2, &buf[1], 1);
    //Load capacitance
    uint8_t a = 0xa5;
    SI44_Write(0x09, &a, 1);
    //GPIO0 & GPIO1 - TX/RX switch
    a = 0x12;
    SI44_Write(0x0b, &a, 1);
    a = 0x15;
    SI44_Write(0x0c, &a, 1);
    
}

void SI44_SetPHConfig(si44_ph_config * conf)
{
    uint8_t buf[3];
    SI44_CalcPHRegisters(*conf, buf);
    SI44_Write(SI44_REG_DATA_ACCESS_CONTROL, &buf[0], 1);
    SI44_Write(SI44_REG_HEADER_CONTROL, &buf[1], 1);
    SI44_Write(SI44_REG_PREAMBULE_LENGTH, &buf[2], 1);
}

void SI44_SetFrequency(float freq)
{
    uint8_t buf[3];
    SI44_CalcFrequencyCarierRegisters(freq, buf);
    SI44_Write(SI44_REG_FREQ_BAND_SELECT, &buf[0], 1);
    SI44_Write(SI44_REG_FREQ_MSB, &buf[1], 1);
    SI44_Write(SI44_REG_FREQ_LSB, &buf[2], 1);
}

void SI44_SetDataRate(uint16_t datarate)
{
    uint8_t buf[2];
    SI44_CalcDataRateRegisters(datarate, buf);
    SI44_Write(SI44_REG_DATARATE_MSB, &buf[0], 1);
    SI44_Write(SI44_REG_DATARATE_LSB, &buf[1], 1);
}

void SI44_SetFrequencyDeviation(uint32_t deviation)
{
    uint8_t d = SI44_CalcFrequencyDeviationRegister(deviation);
    SI44_Write(SI44_REG_DEVIATION, &d, 1);
}

void SI44_SetTXPower(SI44_TX_POWER power)
{
    uint8_t reg_6d;
    //cte aktualni hodnotu
    SI44_Read(SI44_REG_TX_POWER, &reg_6d, 1);
    //vynuluje TX cast
    reg_6d &= 0b11111000;

    reg_6d |= (power & 0b00000111); //ToDo: odstranit pojistku & 0b00000111

    SI44_Write(SI44_REG_TX_POWER, &reg_6d, 1);
}

void SI44_SendPacket(uint8_t * buf, uint8_t length)
{
    uint8_t b;
    SI44_ClearTXFIFO();
    SI44_Read(0x03, &b, 1);
    SI44_Read(0x04, &b, 1);
    SI44_Write(0x3e, &length, 1);
    SI44_Write(SI44_REG_FIFO_ACCESS, buf, length);
    SI44_ResendPacket();
}
void SI44_ResendPacket(void)
{
    uint8_t txon = SI44_OPERATION_MODE_DEFAULT | 0b00001000; //ToDo: musi byt i vysilani
    SI44_Write(SI44_REG_CTRL1, &txon, 1);
}

void SI44_ClearTXFIFO(void)
{
    uint8_t a = 0b00000001;
    SI44_Write(SI44_REG_CTRL2, &a, 1);
    a = 0b00000000;
    SI44_Write(SI44_REG_CTRL2, &a, 1);
}

uint8_t SI44_ReadBatteryVoltage(void)
{
    uint8_t value;
    SI44_Read(SI44_REG_BATTERY_VOLTAGE, &value, 1);
    return value;
}

uint8_t SI44_ReadTemperature(void)
{
    //Set adc input to internal temp. sensor
    uint8_t t = 0;
    SI44_Write(SI44_REG_ADC_CONFIG, &t, 1);
    //Set temperature range
    t = 0b00100000;
    SI44_Write(SI44_REG_TEMP_CONFIG, &t, 1);
    //Trigger ADC conversion
    t = 0b10000000;
    SI44_Write(SI44_REG_ADC_CONFIG, &t, 1);
    //Wait for ADC finish
    HAL_Delay(2);
    //Read out ADC value
    SI44_Read(SI44_REG_ADC_VALUE, &t, 1);
    return t;
}

void SI44_SetSyncBytes(uint8_t * bytes, uint8_t len)
{
    if (len > 4) 
    {
        return;
    }
    SI44_Write(SI44_REG_SYNC3, bytes, len);
}

void SI44_ForceRecalibrate(void)
{
    uint8_t t = SI44_FORCE_RACALIBRATION_VAL;
    SI44_Write(SI44_REG_CALIBRATION, &t, 1);
    HAL_Delay(10);
}
//--------------RX_NEW---------------
void SI44_PresetConfig()
{
	for(uint16_t i = 0; i<sizeof(address);i++)
	{
		SI44_Write(address[i], &byte[i], 1);
		HAL_Delay(1);
	}
}

void SI44_SetRxBandwidth(uint8_t bw_reg)
{
    SI44_Write(SI44_REG_RX_BW, &bw_reg, 1);  // RX_BW register
}

void SI44_SetAGCMode(uint8_t agc_sett)
{
    SI44_Write(SI44_AGC_OVERRIDE_1, &agc_sett, 1);  // RX_BW register
}

void SI44_SetInterrupts1(uint8_t it1)
{
    SI44_Write(SI44_INTERRUPT_ENABLE_1, &it1, 1);  // RX_BW register
}

void SI44_SetInterrupts2(uint8_t it2)
{
    SI44_Write(SI44_INTERRUPT_ENABLE_2, &it2, 1);  // RX_BW register
}

void SI44_ClearRXFIFO(void)
{
    uint8_t a = 0b00000010;
    SI44_Write(SI44_REG_CTRL2, &a, 1);
    a = 0b00000000; //can be set directly, we dont use any of these settings
    SI44_Write(SI44_REG_CTRL2, &a, 1);
}

void SI44_ReadPacket(uint8_t * buf)
{
	uint8_t length=0;
	SI44_Read(SI44_RECEIVED_PACKET_LENGTH, &length, 1); //ToDo: Pokud length neni vetsi jak 64
	SI44_Read(SI44_REG_FIFO_ACCESS, buf, length);

	SI44_ClearRXFIFO();

	uint8_t rxon = SI44_OPERATION_MODE_DEFAULT | 0b00000100; //ToDo: const static?
	SI44_Write(SI44_REG_CTRL1, &rxon, 1);
    //přečíst délku
	//přečíst paket
	//znovu zapnout RX //ToDo: optimalizovat pro spotřebu baterie
	//Z registru se cte tak, ze po docteni jednoho prekroci automaticky na dalsi adresu
}

void SI44_SetRXon()
{
	uint8_t rxon = SI44_OPERATION_MODE_DEFAULT | 0b00000100; //ToDo: const static?
	SI44_Write(SI44_REG_CTRL1, &rxon, 1);
}
