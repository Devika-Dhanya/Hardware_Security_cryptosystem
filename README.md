# Hardware_Security_cryptosystem
# Secure Segmented Parallel-Accumulator (SPA) Polynomial Multiplier

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Double Blind](https://img.shields.io/badge/Review-Double%20Blind-red.svg)](#)

This repository contains the synthesizable Register-Transfer Level (RTL) code, testbenches, and evaluation scripts for the **Secure Segmented Parallel-Accumulator (SPA) Polynomial Multiplier**. 

This hardware accelerator is designed to overcome the severe Electronic Design Automation (EDA) routing congestion and timing violations typically associated with massive polynomial degrees (e.g., $N=1024$) in Post-Quantum Cryptography (PQC). Furthermore, the datapath implements physically isolated accumulator splitting and deferred unmasking to neutralize glitch-induced First-Order Side-Channel Analysis (SCA) vulnerabilities, successfully passing rigorous Test Vector Leakage Assessment (TVLA).

## Repository Structure

To facilitate reproducibility, the repository is organized as follows:

```text
├── hdl/                  # Synthesizable Verilog RTL source files
│   ├── psc.v             
│   ├── sma.v             
│   ├── proposed_unmasked.v
│   └── proposed_masked.v   
├── sim/                  # Simulation environments and testbenches
│   └── tb.v              # Self-checking testbench for functional verification
├── scripts/              # Automation and analysis scripts
│   ├── build.tcl         # Vivado TCL script for automated synthesis/implementation
│   └── tvla_analysis.py  # Python script to compute Welch's t-test statistics
├── LICENSE               # Apache 2.0 Open Source License
└── README.md             # Project documentation

Prerequisites
To reproduce the hardware synthesis and side-channel evaluation results, the following tools are required:

Hardware Synthesis: Xilinx Vivado Design Suite (Version 2018.3 or newer recommended).

Side-Channel Analysis: Python 3.8+ with the following packages installed:  pip install numpy scipy matplotlib

1. Functional Simulation
To verify the mathematical correctness of the negacyclic convolution and deferred unmasking:

Launch Vivado and create a new project, or use the provided TCL script.

Add all Verilog files from the hdl/ directory as design sources.

Add sim/tb_secure_spa.v as the simulation source.

Run the behavioral simulation. The testbench is self-checking and will assert a PASS flag upon successfully validating the streamed polynomial output against expected vectors.

2. FPGA Synthesis and ImplementationTo reproduce the Equivalent Area-Delay Product (EADP) and maximum frequency ($F_{max}$) results reported in the paper, run the automated build script in batch mode from your terminal: vivado -mode batch -source scripts/build.tcl

Note: The build.tcl script is configured by default to target a generic Xilinx UltraScale+ FPGA. You can modify the TARGET_PART variable inside the script to evaluate the architecture on different FPGA families.

3. Side-Channel Evaluation (TVLA)
A Python-based TVLA evaluation script is provided to verify the architecture's glitch-resistance. 

To execute the Welch's t-test analysis, run:    python scripts/tvla_analysis.py --traces data/sample_traces.npy --threshold 4.5

Expected Output: The script will output the maximum absolute t-value ($|t|$) and generate a plot showing the t-statistic across the temporal execution cycles. This confirms that the side-channel leakage remains bounded within the $\pm 4.5$ cryptographic safety threshold, demonstrating first-order SCA immunity.

License
This project is licensed under the Apache License 2.0. See the LICENSE file for details.
