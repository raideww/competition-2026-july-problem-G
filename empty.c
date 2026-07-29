#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define SAMPLE_COUNT     (4096)
#define SAMPLE_RATE_HZ   (2000000UL)
#define ADC_MAX_CODE     (4095)
#define ADC_REFERENCE_MV (3300)
#define MAX_SPECTRAL_COMPONENTS (3U)

int16_t gADCSamples[SAMPLE_COUNT];
int16_t gSamplesMillivolts[SAMPLE_COUNT];
int16_t gDCBiasMillivolts;
uint16_t gUppMillivolts;
uint16_t gUrmsMillivolts;
uint32_t gFundamentalFrequencyHz;
uint8_t gSpectrumComponentCount;
uint32_t gSpectrumFrequencyHz[MAX_SPECTRAL_COMPONENTS];
uint16_t gSpectrumAmplitudeMillivolts[MAX_SPECTRAL_COMPONENTS];
volatile bool gSamplesReady = false;
static volatile bool gCaptureComplete = false;

void captureAndProcessSamples(void)
{
    /* One scratch array plus gADCSamples form an in-place complex FFT. */
    static int16_t fftImaginary[SAMPLE_COUNT];
    static const int32_t twiddleCosQ30[12] = {
        -1073741824,          0,  759250125,  992008094,
         1053110176, 1068571464, 1072448455, 1073418433,
         1073660973, 1073721611, 1073736771, 1073740561
    };
    static const int32_t twiddleSinQ30[12] = {
                  0, -1073741824, -759250125, -410903207,
         -209476638,  -105245103,  -52686014,  -26350943,
          -13176464,    -6588356,   -3294193,   -1647099
    };
    enum {
        FIRST_SPECTRUM_BIN = 20,
        LAST_SPECTRUM_BIN = 1024,
        PEAK_SEPARATION_BINS = 8,
        MIN_SPECTRAL_AMPLITUDE_MV = 1,
        RECONSTRUCTION_POINTS = 2048
    };

    DL_TimerG_stopCounter(SAMPLE_TIMER_INST);
    DL_ADC12_disableConversions(ADC12_0_INST);

    gSamplesReady = false;
    gCaptureComplete = false;
    gUppMillivolts = 0;
    gUrmsMillivolts = 0;
    gFundamentalFrequencyHz = 0;
    gSpectrumComponentCount = 0;
    for (uint32_t i = 0; i < MAX_SPECTRAL_COMPONENTS; i++) {
        gSpectrumFrequencyHz[i] = 0;
        gSpectrumAmplitudeMillivolts[i] = 0;
    }

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

    int16_t minimumMillivolts = 32767;
    int16_t maximumMillivolts = -32768;
    uint64_t sumOfSquares = 0;
    uint16_t maximumAbsoluteMillivolts = 0;

    for (int32_t i = 0; i < SAMPLE_COUNT; i++) {
        int32_t delta = (int32_t) gADCSamples[i] - biasCode;
        int32_t numerator = delta * ADC_REFERENCE_MV;

        numerator += (numerator >= 0) ? ADC_MAX_CODE / 2
                                     : -(ADC_MAX_CODE / 2);
        int16_t sampleMillivolts =
            (int16_t) (numerator / ADC_MAX_CODE);
        gSamplesMillivolts[i] = sampleMillivolts;

        if (sampleMillivolts < minimumMillivolts) {
            minimumMillivolts = sampleMillivolts;
        }
        if (sampleMillivolts > maximumMillivolts) {
            maximumMillivolts = sampleMillivolts;
        }

        int32_t absoluteMillivolts = sampleMillivolts;
        if (absoluteMillivolts < 0) {
            absoluteMillivolts = -absoluteMillivolts;
        }
        if ((uint32_t) absoluteMillivolts > maximumAbsoluteMillivolts) {
            maximumAbsoluteMillivolts = (uint16_t) absoluteMillivolts;
        }
        sumOfSquares +=
            (uint64_t) ((int32_t) sampleMillivolts * sampleMillivolts);
    }

    gUppMillivolts = (uint16_t) ((int32_t) maximumMillivolts -
                                  minimumMillivolts);

    gUrmsMillivolts = (uint16_t) (sqrtf(
        (float) sumOfSquares / SAMPLE_COUNT) + 0.5f);

    /*
     * Apply a periodic Hann window. Scaling the input close to int16_t's
     * range preserves sub-millivolt FFT precision despite stage scaling.
     * gADCSamples is no longer needed after conversion, so it is FFT scratch.
     */
    uint16_t fftInputScale = 1;
    if (maximumAbsoluteMillivolts != 0) {
        fftInputScale = (uint16_t) (30000U /
                                    maximumAbsoluteMillivolts);
        if (fftInputScale == 0) {
            fftInputScale = 1;
        }
    }

    int32_t windowCosQ30 = 1073741824;
    int32_t windowSinQ30 = 0;
    for (int32_t i = 0; i < SAMPLE_COUNT; i++) {
        int32_t windowQ15 =
            (int32_t) ((1073741824LL - windowCosQ30) >> 16);
        if (windowQ15 < 0) {
            windowQ15 = 0;
        }
        if (windowQ15 > 32767) {
            windowQ15 = 32767;
        }

        int32_t windowed =
            (int32_t) gSamplesMillivolts[i] * fftInputScale * windowQ15;
        windowed += (windowed >= 0) ? 16384 : -16384;
        gADCSamples[i] = (int16_t) (windowed / 32768);
        fftImaginary[i] = 0;

        int64_t nextCos =
            (int64_t) windowCosQ30 * 1073740561 -
            (int64_t) windowSinQ30 * 1647099;
        int64_t nextSin =
            (int64_t) windowSinQ30 * 1073740561 +
            (int64_t) windowCosQ30 * 1647099;
        windowCosQ30 = (int32_t) ((nextCos + (1LL << 29)) >> 30);
        windowSinQ30 = (int32_t) ((nextSin + (1LL << 29)) >> 30);
    }

    /* Bit-reverse, then run a radix-2 FFT scaled by two at every stage. */
    uint32_t reversed = 0;
    for (uint32_t i = 1; i < SAMPLE_COUNT; i++) {
        uint32_t bit = SAMPLE_COUNT >> 1;
        while ((reversed & bit) != 0) {
            reversed ^= bit;
            bit >>= 1;
        }
        reversed ^= bit;

        if (i < reversed) {
            int16_t temporary = gADCSamples[i];
            gADCSamples[i] = gADCSamples[reversed];
            gADCSamples[reversed] = temporary;
        }
    }

    uint32_t fftSize = 2;
    for (uint32_t stage = 0; stage < 12; stage++) {
        uint32_t halfSize = fftSize >> 1;
        int32_t twiddleRealQ30 = 1073741824;
        int32_t twiddleImaginaryQ30 = 0;

        for (uint32_t offset = 0; offset < halfSize; offset++) {
            for (uint32_t i = offset; i < SAMPLE_COUNT; i += fftSize) {
                uint32_t lower = i + halfSize;
                int64_t productReal =
                    (int64_t) twiddleRealQ30 * gADCSamples[lower] -
                    (int64_t) twiddleImaginaryQ30 *
                        fftImaginary[lower];
                int64_t productImaginary =
                    (int64_t) twiddleRealQ30 * fftImaginary[lower] +
                    (int64_t) twiddleImaginaryQ30 *
                        gADCSamples[lower];
                int32_t lowerReal =
                    (int32_t) ((productReal + (1LL << 29)) >> 30);
                int32_t lowerImaginary =
                    (int32_t) ((productImaginary + (1LL << 29)) >> 30);
                int32_t upperReal = gADCSamples[i];
                int32_t upperImaginary = fftImaginary[i];

                gADCSamples[i] =
                    (int16_t) ((upperReal + lowerReal) / 2);
                fftImaginary[i] =
                    (int16_t) ((upperImaginary + lowerImaginary) / 2);
                gADCSamples[lower] =
                    (int16_t) ((upperReal - lowerReal) / 2);
                fftImaginary[lower] =
                    (int16_t) ((upperImaginary - lowerImaginary) / 2);
            }

            int64_t nextReal =
                (int64_t) twiddleRealQ30 * twiddleCosQ30[stage] -
                (int64_t) twiddleImaginaryQ30 * twiddleSinQ30[stage];
            int64_t nextImaginary =
                (int64_t) twiddleImaginaryQ30 * twiddleCosQ30[stage] +
                (int64_t) twiddleRealQ30 * twiddleSinQ30[stage];
            twiddleRealQ30 =
                (int32_t) ((nextReal + (1LL << 29)) >> 30);
            twiddleImaginaryQ30 =
                (int32_t) ((nextImaginary + (1LL << 29)) >> 30);
        }
        fftSize <<= 1;
    }

    /* Find at most three separated local maxima. */
    uint16_t selectedBin[MAX_SPECTRAL_COMPONENTS] = {0};
    while (gSpectrumComponentCount < MAX_SPECTRAL_COMPONENTS) {
        uint32_t bin = 0;
        uint64_t strongestPower = 0;
        for (uint32_t currentBin = FIRST_SPECTRUM_BIN;
             currentBin <= LAST_SPECTRUM_BIN; currentBin++) {
            bool separated = true;
            for (uint32_t i = 0; i < gSpectrumComponentCount; i++) {
                uint32_t distance = (currentBin > selectedBin[i])
                                        ? currentBin - selectedBin[i]
                                        : selectedBin[i] - currentBin;
                if (distance <= PEAK_SEPARATION_BINS) {
                    separated = false;
                    break;
                }
            }
            if (!separated) {
                continue;
            }

            int32_t real = gADCSamples[currentBin];
            int32_t imaginary = fftImaginary[currentBin];
            uint64_t power = (uint64_t) (real * real) +
                             (uint64_t) (imaginary * imaginary);
            real = gADCSamples[currentBin - 1];
            imaginary = fftImaginary[currentBin - 1];
            uint64_t leftPower = (uint64_t) (real * real) +
                                 (uint64_t) (imaginary * imaginary);
            real = gADCSamples[currentBin + 1];
            imaginary = fftImaginary[currentBin + 1];
            uint64_t rightPower = (uint64_t) (real * real) +
                                  (uint64_t) (imaginary * imaginary);

            if ((power >= leftPower) && (power > rightPower) &&
                (power > strongestPower)) {
                strongestPower = power;
                bin = currentBin;
            }
        }
        if (bin == 0) {
            break;
        }

        float magnitudes[3];
        for (int32_t neighbor = -1; neighbor <= 1; neighbor++) {
            int32_t real = gADCSamples[bin + neighbor];
            int32_t imaginary = fftImaginary[bin + neighbor];
            float power = (float) (real * real) +
                          (float) (imaginary * imaginary);
            magnitudes[neighbor + 1] = sqrtf(power);
        }

        float magnitudeDenominator =
            magnitudes[0] + 2.0f * magnitudes[1] + magnitudes[2];
        float fractionalBin = 0.0f;
        if (magnitudeDenominator != 0.0f) {
            fractionalBin =
                2.0f * (magnitudes[2] - magnitudes[0]) /
                magnitudeDenominator;
        }
        if (fractionalBin < -0.5f) {
            fractionalBin = -0.5f;
        } else if (fractionalBin > 0.5f) {
            fractionalBin = 0.5f;
        }
        uint32_t frequencyHz = (uint32_t) (
            ((bin + fractionalBin) * SAMPLE_RATE_HZ / SAMPLE_COUNT) +
            0.5f);
        if (frequencyHz < 10000U) {
            frequencyHz = 10000U;
        } else if (frequencyHz > 500000U) {
            frequencyHz = 500000U;
        }

        uint64_t lobePower = 0;
        for (int32_t neighbor = -2; neighbor <= 2; neighbor++) {
            int32_t real = gADCSamples[bin + neighbor];
            int32_t imaginary = fftImaginary[bin + neighbor];
            lobePower +=
                (uint64_t) (real * real) +
                (uint64_t) (imaginary * imaginary);
        }
        uint16_t amplitudeMillivolts = (uint16_t) (sqrtf(
            (32.0f * (float) lobePower) /
            (3.0f * fftInputScale * fftInputScale)) + 0.5f);
        if (amplitudeMillivolts < MIN_SPECTRAL_AMPLITUDE_MV) {
            break;
        }

        uint32_t result = gSpectrumComponentCount;
        selectedBin[result] = (uint16_t) bin;
        gSpectrumFrequencyHz[result] = frequencyHz;
        gSpectrumAmplitudeMillivolts[result] = amplitudeMillivolts;
        gSpectrumComponentCount++;
    }

    /* Frequency order makes the first line the fundamental. */
    for (uint32_t i = 1; i < gSpectrumComponentCount; i++) {
        uint32_t frequency = gSpectrumFrequencyHz[i];
        uint16_t amplitude = gSpectrumAmplitudeMillivolts[i];
        uint32_t position = i;
        while ((position > 0) &&
               (gSpectrumFrequencyHz[position - 1] > frequency)) {
            gSpectrumFrequencyHz[position] =
                gSpectrumFrequencyHz[position - 1];
            gSpectrumAmplitudeMillivolts[position] =
                gSpectrumAmplitudeMillivolts[position - 1];
            position--;
        }
        gSpectrumFrequencyHz[position] = frequency;
        gSpectrumAmplitudeMillivolts[position] = amplitude;
    }
    if (gSpectrumComponentCount != 0) {
        gFundamentalFrequencyHz = gSpectrumFrequencyHz[0];

        /* Recover each line's phase from the preserved time samples. */
        float componentCos[MAX_SPECTRAL_COMPONENTS] = {0.0f};
        float componentSin[MAX_SPECTRAL_COMPONENTS] = {0.0f};
        float oscillatorCos[MAX_SPECTRAL_COMPONENTS];
        float oscillatorSin[MAX_SPECTRAL_COMPONENTS];
        float stepCos[MAX_SPECTRAL_COMPONENTS];
        float stepSin[MAX_SPECTRAL_COMPONENTS];
        const float twoPi = 6.28318530718f;

        for (uint32_t i = 0; i < gSpectrumComponentCount; i++) {
            float angle = twoPi * gSpectrumFrequencyHz[i] /
                          SAMPLE_RATE_HZ;
            oscillatorCos[i] = 1.0f;
            oscillatorSin[i] = 0.0f;
            stepCos[i] = cosf(angle);
            stepSin[i] = sinf(angle);
        }

        windowCosQ30 = 1073741824;
        windowSinQ30 = 0;
        for (uint32_t sample = 0; sample < SAMPLE_COUNT; sample++) {
            int32_t windowQ15 =
                (int32_t) ((1073741824LL - windowCosQ30) >> 16);
            if (windowQ15 < 0) {
                windowQ15 = 0;
            } else if (windowQ15 > 32767) {
                windowQ15 = 32767;
            }
            float windowedSample =
                (float) gSamplesMillivolts[sample] * windowQ15 / 32768.0f;

            for (uint32_t i = 0; i < gSpectrumComponentCount; i++) {
                componentCos[i] += windowedSample * oscillatorCos[i];
                componentSin[i] += windowedSample * oscillatorSin[i];
                float nextOscillatorCos =
                    oscillatorCos[i] * stepCos[i] -
                    oscillatorSin[i] * stepSin[i];
                oscillatorSin[i] =
                    oscillatorSin[i] * stepCos[i] +
                    oscillatorCos[i] * stepSin[i];
                oscillatorCos[i] = nextOscillatorCos;
            }

            int64_t nextWindowCos =
                (int64_t) windowCosQ30 * 1073740561 -
                (int64_t) windowSinQ30 * 1647099;
            int64_t nextWindowSin =
                (int64_t) windowSinQ30 * 1073740561 +
                (int64_t) windowCosQ30 * 1647099;
            windowCosQ30 =
                (int32_t) ((nextWindowCos + (1LL << 29)) >> 30);
            windowSinQ30 =
                (int32_t) ((nextWindowSin + (1LL << 29)) >> 30);
        }

        float sumOfComponentSquares = 0.0f;
        for (uint32_t i = 0; i < gSpectrumComponentCount; i++) {
            float phaseLength = sqrtf(componentCos[i] * componentCos[i] +
                                      componentSin[i] * componentSin[i]);
            float amplitude = gSpectrumAmplitudeMillivolts[i];
            if (phaseLength > 0.0f) {
                componentCos[i] *= amplitude / phaseLength;
                componentSin[i] *= amplitude / phaseLength;
            } else {
                componentCos[i] = amplitude;
                componentSin[i] = 0.0f;
            }
            sumOfComponentSquares += amplitude * amplitude;

            uint32_t harmonic =
                (gSpectrumFrequencyHz[i] +
                 (gFundamentalFrequencyHz / 2)) /
                gFundamentalFrequencyHz;
            if (harmonic == 0) {
                harmonic = 1;
            }
            float angle = twoPi * harmonic / RECONSTRUCTION_POINTS;
            oscillatorCos[i] = 1.0f;
            oscillatorSin[i] = 0.0f;
            stepCos[i] = cosf(angle);
            stepSin[i] = sinf(angle);
        }

        float reconstructedMinimum = 0.0f;
        float reconstructedMaximum = 0.0f;
        for (uint32_t point = 0; point < RECONSTRUCTION_POINTS; point++) {
            float reconstructedSample = 0.0f;
            for (uint32_t i = 0; i < gSpectrumComponentCount; i++) {
                reconstructedSample +=
                    componentCos[i] * oscillatorCos[i] +
                    componentSin[i] * oscillatorSin[i];
                float nextOscillatorCos =
                    oscillatorCos[i] * stepCos[i] -
                    oscillatorSin[i] * stepSin[i];
                oscillatorSin[i] =
                    oscillatorSin[i] * stepCos[i] +
                    oscillatorCos[i] * stepSin[i];
                oscillatorCos[i] = nextOscillatorCos;
            }
            if ((point == 0) ||
                (reconstructedSample < reconstructedMinimum)) {
                reconstructedMinimum = reconstructedSample;
            }
            if ((point == 0) ||
                (reconstructedSample > reconstructedMaximum)) {
                reconstructedMaximum = reconstructedSample;
            }
        }

        gUppMillivolts = (uint16_t) (
            reconstructedMaximum - reconstructedMinimum + 0.5f);
        gUrmsMillivolts = (uint16_t) (
            sqrtf(0.5f * sumOfComponentSquares) + 0.5f);
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

        // DL_Common_delayCycles(CPUCLK_FREQ);
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
