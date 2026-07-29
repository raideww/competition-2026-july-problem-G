#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define SAMPLE_COUNT (4096U)

uint16_t gADCSamples[SAMPLE_COUNT];
volatile bool gADCSamplesReady = false;

int main(void)
{
    SYSCFG_DL_init();

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t) DL_ADC12_getMemResultAddress(
            ADC12_0_INST, ADC12_0_ADCMEM_0));
    DL_DMA_setDestAddr(
        DMA, DMA_CH0_CHAN_ID, (uint32_t) &gADCSamples[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, SAMPLE_COUNT);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    DL_ADC12_enableDMA(ADC12_0_INST);
    DL_ADC12_enableConversions(ADC12_0_INST);
    DL_TimerG_startCounter(SAMPLE_TIMER_INST);

    while (!gADCSamplesReady) {
        __WFE();
    }

    while (1) {
        __WFI();
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC12_0_INST) ==
        DL_ADC12_IIDX_DMA_DONE) {
        DL_TimerG_stopCounter(SAMPLE_TIMER_INST);
        DL_ADC12_disableConversions(ADC12_0_INST);
        gADCSamplesReady = true;
    }
}
