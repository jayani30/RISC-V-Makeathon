# Risc-v-Makeathon                                                                                                              

## RISC-V-Based CRC-8 Error Detection System on FPGA

A Verilog-based System-on-Chip (SoC) designed for programmable sonar signal generation in Autonomous Underwater Vehicles (AUVs). The system integrates the PicoRV32 RISC-V processor with custom peripherals and digital signal-generation modules, targeting the **BooleanBoard FPGA platform**.

## Abstract

This project aims to develop a compact, programmable FPGA-based SoC for sonar signal generation. It uses the PicoRV32 RISC-V processor to control memory and peripheral modules, along with dedicated hardware for CRC-8 computation, sine waveform sample generation, and Binary Phase Shift Keying (BPSK).

The design combines software programmability with hardware acceleration to provide a flexible platform for digital signal generation. The system is developed using Verilog HDL, with C firmware used to control supported peripherals. The architecture is intended for simulation, FPGA implementation, and future enhancement with configurable adaptive chirp generation.

## Objectives

- Integrate the PicoRV32 RISC-V processor into a custom SoC.
- Implement RAM, ROM, and address decoding.
- Develop UART communication and digital input registers.
- Implement a hardware CRC-8 accelerator.
- Generate sinusoidal samples using a sine lookup table (LUT).
- Implement BPSK sample generation.
- Verify the design through simulation and FPGA testing.

## Key Features

- PicoRV32 RISC-V processor integration.
- Custom memory-mapped SoC architecture.
- UART communication interface.
- Digital input register.
- Hardware CRC-8 accelerator.
- Sine LUT for digital waveform generation.
- BPSK sample-generation module.
- Modular Verilog RTL and testbench development.

## Block Diagram

![RISC-V Adaptive Chirp Sonar Transmitter SoC](https://github.com/jayani30/RISC-V-Makeathon/blob/1bc29a4ec7df613ff06910278f1c85f902cf0108/jpeg.jpg)

## Module Description

| Module | Description |
|---|---|
| `top.v` | Top-level integration of the system components. |
| `picorv32_top.v` | Integrates the PicoRV32 processor with the required interfaces. |
| `picorv32_soc_top.v` | Connects the processor, memories, address decoder, and SoC peripherals. |
| RAM / ROM | Provides instruction and data storage. |
| Address Decoder | Selects the appropriate memory or peripheral based on the address. |
| UART | Supports serial communication and debugging. |
| Digital Input Register | Reads external digital input signals. |
| CRC-8 Accelerator | Performs hardware-based CRC-8 calculations. |
| Sine LUT | Provides digital sinusoidal sample values. |
| BPSK Generator | Generates phase-modulated digital samples. |
| Testbenches | Verify module functionality through simulation. |



## Working Principle

1. The PicoRV32 processor executes the firmware and controls the SoC.
2. The address decoder routes memory and peripheral accesses to the appropriate modules.
3. UART and digital input registers provide communication and input-handling capabilities.
4. The CRC-8 accelerator computes error-detection values for supported data.
5. The sine LUT supplies digital waveform samples, while the BPSK module generates phase-modulated samples.
6. The design is verified using Verilog testbenches before FPGA implementation.

Adaptive chirp generation requires additional frequency-sweep control logic if it is not already implemented.

## Hardware and Software Requirements

**Hardware**
- BooleanBoard FPGA platform
- Host computer
- Required FPGA programming interface

**Software**
- Verilog HDL
- Compatible FPGA synthesis and simulation tools
- RISC-V GCC toolchain
- C firmware

## Applications

- FPGA-based sonar signal-generation research.
- Underwater robotics and AUV development.
- Digital waveform-generation experiments.
- RISC-V processor and peripheral integration.
- Embedded hardware-software co-design.

## Future Enhancements

- Configurable chirp frequency and sweep duration.
- Runtime waveform control through UART.
- DAC integration for analog signal output.
- FPGA resource and timing optimization.
- Hardware validation of generated waveforms.

## Reference

[PicoRV32 RISC-V Processor – YosysHQ](https://github.com/YosysHQ/picorv32)

## Project Status

**Under Development**

The project is undergoing RTL integration, simulation, and FPGA validation. Further development will focus on verifying waveform generation and extending the design toward adaptive chirp transmission.

## Technology Stack

- **HDL:** Verilog
- **Processor:** PicoRV32 RISC-V
- **Firmware:** C
- **Platform:** BooleanBoard FPGA
- **Tools:** FPGA development and simulation tools                                                                                                                                            
