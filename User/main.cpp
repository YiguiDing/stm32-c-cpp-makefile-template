extern "C" {
#include "stm32f10x.h"
#include "Delay.h"
}

int main(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct = {GPIO_Pin_13, GPIO_Speed_50MHz, GPIO_Mode_Out_PP};
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    while (1)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        Delay_ms(15);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        Delay_ms(15);
    }
}
