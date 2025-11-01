#include "SI4432_IO.h"
//#include "stm32f1xx_hal_spi.h"
SPI_HandleTypeDef * spi_interface;
GPIO_TypeDef * nss_port;
uint16_t nss_pin;

void SI44_Read(uint8_t reg, uint8_t * buf, uint8_t length) //ToDo: replace with more secure function
{
    uint8_t b[length + 1];
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_interface, &reg, b, length+1, 100); //s prvnim taktem sbernice sice nejprve prijme neplatna data, ale nevypina NSS a neprepina CLK -> receive primo navazuje a v druhem kroku uz ma spravne
    //S kazdou hranou hodinoveho signalu se posle jeden bit dat; CPOL - data se meni na falling, ctou na rising
    //První bajt kterej přijme je blbost, druhej bajt kterej odešle je blbost -> pozor
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
    for (int i = 0; i < length; i++) //proc tu neni memcpy
    {
        buf[i] = b[i+1];
    }
    
}

/*void SI44_Read(uint8_t reg, uint8_t * buf, uint8_t length)
{
    // Ujistěte se, že MSB (write bit) je 0 pro čtení.
    // Vaše volání (0x6E) to už splňuje, ale pro jistotu:
    reg = reg & 0b01111111;

    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);

    // 1. Odešli 1 byte - adresu registru, který chceme číst
    HAL_SPI_Transmit(spi_interface, &reg, 1, 100);

    // 2. Přijmi 'length' bytů dat.
    //    Během příjmu bude STM32 automaticky odesílat "dummy" byty
    //    (obvykle 0x00 nebo 0xFF), aby generovalo hodinový signál.
    HAL_SPI_Receive(spi_interface, buf, length, 100);

    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

    // Data jsou nyní přímo v 'buf', není potřeba žádná kopírovací smyčka.
}*//*
void SI44_Read(uint8_t reg, uint8_t * buf, uint8_t length)
{
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);

    // nejdřív pošlu adresu registru
    HAL_SPI_Transmit(spi_interface, &reg, 1, 100);

    // pak přečtu data
    HAL_SPI_Receive(spi_interface, buf, length, 100);

    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
}*/


void SI44_Write(uint8_t reg, uint8_t * buf, uint8_t length)
{
    uint8_t b[length + 1]; //ToDo: allocate non variable; ToDo: memcpy
    b[0] = reg | 0b10000000;
    for (int i = 0; i < length; i++)
    {
        b[i+1] = buf[i];
    }
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_interface, b, length + 1, 100);
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
}

void SI44_IO_Init(SPI_HandleTypeDef * hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    spi_interface = hspi;
    nss_port = GPIOx;
    nss_pin = GPIO_Pin;
}
