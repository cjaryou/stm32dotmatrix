#include "stm32f4xx.h"        // STM32F4 için CMSIS başlık dosyası
#include "system_stm32f4xx.h" // Sistem başlık dosyası
#include "fonts.h"            // Harf ve sayı tabloları

#define CS_PIN GPIO_PIN_0
#define CS_PORT GPIOA

void delay_ms(uint32_t ms)
{
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // 1 ms için değer
    SysTick->VAL = 0;                             // Sayaç sıfırlanır
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    for (uint32_t i = 0; i < ms; i++)
    {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
            ;
    }
    SysTick->CTRL = 0; // Sayaç kapatılır
}

void SPI1_Init(void)
{
    // SPI1 için clock'u aktif et
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // GPIOA Pin 5 (SCK), Pin 7 (MOSI) ayarları
    GPIOA->MODER |= (2U << GPIO_MODER_MODE5_Pos) | (2U << GPIO_MODER_MODE7_Pos);
    GPIOA->AFR[0] |= (5U << GPIO_AFRL_AFSEL5_Pos) | (5U << GPIO_AFRL_AFSEL7_Pos);

    // GPIOA Pin 0 (CS) ayarları
    GPIOA->MODER |= (1U << GPIO_MODER_MODE0_Pos);
    GPIOA->ODR |= CS_PIN; // CS high

    // SPI1 ayarları
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0; // Master mod, Baudrate f_PCLK/4
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // Yazılım NSS yönetimi
    SPI1->CR1 |= SPI_CR1_SPE;                // SPI aktif
}

void SPI1_Transmit(uint8_t address, uint8_t data)
{
    GPIOA->ODR &= ~CS_PIN; // CS low
    while (!(SPI1->SR & SPI_SR_TXE))
        ;               // TXE (Transmit Empty) bekle
    SPI1->DR = address; // Adres gönder
    while (!(SPI1->SR & SPI_SR_TXE))
        ;
    SPI1->DR = data; // Veri gönder
    while (!(SPI1->SR & SPI_SR_TXE))
        ;
    while (SPI1->SR & SPI_SR_BSY)
        ;                 // SPI meşgul değil mi kontrol et
    GPIOA->ODR |= CS_PIN; // CS high
}

void MAX7219_Init(void)
{
    SPI1_Transmit(0x0F, 0x00); // Display test off
    SPI1_Transmit(0x0C, 0x01); // Shutdown register: normal operation
    SPI1_Transmit(0x0B, 0x07); // Scan limit: all rows
    SPI1_Transmit(0x09, 0x00); // Decode mode: no decode
    SPI1_Transmit(0x0A, 0x08); // Intensity: medium
}

void Display_Character(const uint8_t *character)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        SPI1_Transmit(i + 1, character[i]);
    }
}

int main(void)
{
    SystemInit();   // Sistem yapılandırması
    SPI1_Init();    // SPI başlatma
    MAX7219_Init(); // MAX7219 başlatma

    while (1)
    {
        // 0-9 sayıcı
        for (uint8_t i = 0; i < 10; i++)
        {
            Display_Character(Font_Numbers[i]);
            delay_ms(1000);
        }

        // a-z harfleri
        for (uint8_t i = 0; i < 26; i++)
        {
            Display_Character(Font_Lowercase[i]);
            delay_ms(1000);
        }

        // A-Z harfleri
        for (uint8_t i = 0; i < 26; i++)
        {
            Display_Character(Font_Uppercase[i]);
            delay_ms(1000);
        }
    }
}
