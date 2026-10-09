# Risc-v-Makeathon                                                                                                                                                    # BooleanBoard

## RISC-V Adaptive Chirp Sonar Transmitter SoC for Low-Power AUV Payloads

A custom System-on-Chip (SoC) developed using Verilog HDL and C firmware, based on the PicoRV32 RISC-V processor and targeting the Xilinx ZedBoard FPGA.

### Features

* PicoRV32 RISC-V processor
* Custom SoC integration with RAM/ROM and address decoding
* UART communication
* Digital input register
* Hardware CRC-8 accelerator
* Sine lookup table (LUT)
* BPSK sample generation
* Adaptive chirp signal-generation development

### Technologies

* Verilog HDL
* C
* RISC-V GCC
* Xilinx Vivado
* ZedBoard (Zynq-7000)

### Objective

To explore a low-power, FPGA-based embedded SoC for configurable sonar signal generation in autonomous underwater vehicle (AUV) applications.

### Reference

[PicoRV32 RISC-V Core](https://github.com/YosysHQ/picorv32)

### Status

Under development. Features and hardware functionality are subject to simulation and FPGA verification.
