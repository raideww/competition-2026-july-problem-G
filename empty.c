#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define SAMPLE_COUNT     (4096)
#define ADC_MAX_CODE     (4095)
#define ADC_REFERENCE_MV (3300)

int16_t gADCSamples[SAMPLE_COUNT];
int16_t gSamplesMillivolts[SAMPLE_COUNT];
int16_t gDCBiasMillivolts;
volatile bool gSamplesReady = false;
static volatile bool gCaptureComplete = false;

void captureAndProcessSamples(void)
{
    DL_TimerG_stopCounter(SAMPLE_TIMER_INST);
    DL_ADC12_disableConversions(ADC12_0_INST);

    gSamplesReady = false;
    gCaptureComplete = false;

    DL_ADC12_clearInterruptStatus(
        ADC12_0_INST, DL_ADC12_INTERRUPT_DMA_DONE);
    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t) DL_ADC12_getMemResultAddress(
            ADC12_0_INST, ADC12_0_ADCMEM_0));
    DL_DMA_setDestAddr(
        DMA, DMA_CH0_CHAN_ID, (uint32_t) &gADCSamples[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, SAMPLE_COUNT);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    DL_ADC12_enableDMA(ADC12_0_INST);
    DL_ADC12_enableConversions(ADC12_0_INST);
    DL_TimerG_setTimerCount(SAMPLE_TIMER_INST, SAMPLE_TIMER_INST_LOAD_VALUE);
    DL_TimerG_startCounter(SAMPLE_TIMER_INST);

    while (!gCaptureComplete) {
        __WFE();
    }

    int32_t sum = 0;
    for (int32_t i = 0; i < SAMPLE_COUNT; i++) {
        sum += gADCSamples[i];
    }

    int32_t biasCode = (sum + (SAMPLE_COUNT / 2)) / SAMPLE_COUNT;
    gDCBiasMillivolts = (int16_t) (((biasCode * ADC_REFERENCE_MV) +
                                    (ADC_MAX_CODE / 2)) /
                                   ADC_MAX_CODE);

    for (int32_t i = 0; i < SAMPLE_COUNT; i++) {
        int32_t delta = (int32_t) gADCSamples[i] - biasCode;
        int32_t numerator = delta * ADC_REFERENCE_MV;

        numerator += (numerator >= 0) ? ADC_MAX_CODE / 2
                                     : -(ADC_MAX_CODE / 2);
        gSamplesMillivolts[i] = (int16_t) (numerator / ADC_MAX_CODE);
    }
    gSamplesReady = true;
}

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    while (1) {
        captureAndProcessSamples();

        DL_Common_delayCycles(CPUCLK_FREQ);
    }
}

void ADC12_0_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC12_0_INST) ==
        DL_ADC12_IIDX_DMA_DONE) {
        DL_TimerG_stopCounter(SAMPLE_TIMER_INST);
        DL_ADC12_disableConversions(ADC12_0_INST);
        gCaptureComplete = true;
    }
}
