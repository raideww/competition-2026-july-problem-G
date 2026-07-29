# Periodic Signal Measurement and Analysis Device (Problem G)

This project implements a periodic-signal measurement and analysis device for
Problem G of the 2026 National Undergraduate Electronics Design Contest
(TI Cup). The device measures a periodic voltage signal from a source with a
50 ohm output impedance, displays its waveform or voltage spectrum, and reports
the required time-domain and frequency-domain parameters.

## Hardware and Development Platform

- Target microcontroller: **Texas Instruments MSPM0G3507**
- Development board: [LP-MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507)
- Firmware environment: Code Composer Studio, DriverLib, and SysConfig
- Power input: single regulated 5 V DC supply
- Signal input: board-mounted BNC socket for a 50 ohm BNC cable
- Display: at least 6 inches

## Core Functional Requirements

The device shall:

1. Measure and analyze periodic signals composed of a fundamental component and
   one or two harmonic components.
2. Provide a frequency resolution of 500 Hz.
3. Use a key-controlled selection to display either one or three complete signal
   periods on a single screen.
4. Display the peak-to-peak voltage (`Upp`) and true RMS voltage (`Urms`) with an
   absolute error no greater than 5 mV.
5. Display the fundamental frequency with an absolute error no greater than
   1 kHz.
6. Qualitatively display the voltage spectrum on the positive-frequency axis.
   The number, relative frequency positions, and relative heights of the
   spectral lines shall approximately match the input signal.
7. Display the amplitude of each voltage-spectrum component with an absolute
   error no greater than 5 mV.
8. Complete each requested measurement or display operation within 2 seconds
   after startup.

## Test Scenarios

### 1. Signal `ua`

| Parameter | Requirement |
| --- | --- |
| Peak-to-peak voltage | 100 mV to 250 mV |
| Frequency range of all components | 10 kHz to 200 kHz |
| Required outputs | One or three complete periods, `Upp`, true `Urms`, fundamental frequency, qualitative spectrum, and each spectral-component amplitude |

### 2. Signal `ub`

| Parameter | Requirement |
| --- | --- |
| Peak-to-peak voltage | 50 mV to 250 mV |
| Frequency range of all components | 10 kHz to 500 kHz |
| Required outputs | One or three complete periods, `Upp`, true `Urms`, fundamental frequency, qualitative spectrum, and each spectral-component amplitude |

### 3. Signal `ub` with high-frequency interference

The input is `u = ub + uJ`. The device must suppress the influence of the
single-frequency interference signal `uJ` and report the required measurements
for `ub`.

| Parameter | Requirement |
| --- | --- |
| `ub` peak-to-peak voltage | 50 mV to 250 mV |
| Frequency range of all `ub` components | 10 kHz to 500 kHz |
| `uJ` peak-to-peak voltage | 200 mV |
| `uJ` frequency | At least 1 MHz |
| Required outputs for `ub` | One or three complete periods, `Upp`, true `Urms`, fundamental frequency, qualitative spectrum, and each spectral-component amplitude |

## Interface and Test Constraints

- The signal source and its output cable have a 50 ohm impedance.
- Both ends of the signal cable use BNC connectors, and the device PCB must
  provide a BNC input socket.
- Spectrum results shall list or display the positive-frequency components and
  their amplitudes. For example, a signal containing 10.5 kHz, 31.5 kHz, and
  42.0 kHz components shall show those three frequencies with their respective
  amplitudes.
- A harmonic-generator-capable signal generator may be used to produce the
  fundamental and harmonic components for verification.

## Design Report Requirements

The final report should include:

- comparison and justification of the selected design approach;
- theoretical analysis and calculation of system parameters;
- system architecture, block diagram, circuit schematics, firmware design, and
  software flowchart;
- a complete test plan, results, and analysis; and
- a clear abstract, well-structured body, and complete and accurate figures and
  tables.

## Signal Acquisition

The ADC and DMA acquisition path is configured as follows:

| Item | Configuration |
| --- | --- |
| Analog input | `PA25` / LaunchPad `J1_2`, ADC0 channel 2 |
| ADC format | 12-bit unsigned, VDDA reference |
| Sample rate | 2.000 MS/s, triggered by TIMG0 every 500 ns |
| Capture size | 4096 samples (`uint16_t`, 8 KiB) |
| Capture duration | 2.048 ms |
| FFT bin spacing | 488.28125 Hz for a 4096-point FFT |
| DMA | ADC0 MEM0 to `gADCSamples`, incrementing destination |

At startup, firmware captures one block and sets `gADCSamplesReady`. The timer
then stops so `gADCSamples` remains stable for waveform and spectrum processing.

`PA25` must stay between 0 V and VDDA. The competition signal therefore needs
an analog front end that provides 50-ohm termination, gain, mid-supply bias,
input protection, and anti-alias filtering before the ADC pin.
Accurate voltage results require measuring the real reference and calibrating
front-end gain and offset.

Signal analysis, interference rejection, display control, and user-interface
functions remain to be implemented.
