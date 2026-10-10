# Risc-v-Makeathon                                                                                                                                                 

### RISC-V Adaptive Chirp Sonar Transmitter SoC

A Verilog-based SoC designed for low-power sonar signal generation in Autonomous Underwater Vehicles (AUVs), using the PicoRV32 RISC-V processor and targeting the **BooleanBoard FPGA platform**.

## Features

* PicoRV32 RISC-V processor
* Custom SoC with RAM/ROM and address decoding
* UART communication
* Digital input register
* Hardware CRC-8 accelerator
* Sine lookup table (LUT)
* BPSK sample generation

## Tech Stack ![SoC Architecture](RISC-V/soc_architecture.png)


* **Hardware:** Verilog HDL, BooleanBoard FPGA
* **Firmware:** C, RISC-V GCC
* **Tools:** FPGA development tools

## Objective

To develop a compact, low-power FPGA-based SoC for programmable sonar signal generation in AUV applications.

## Reference

[PicoRV32 RISC-V Core](https://github.com/YosysHQ/picorv32)

## Status

Under development. Functionality is subject to simulation and FPGA validation.
