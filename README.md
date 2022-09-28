# DynPath - A Non-intrusive Hardware-based Execution Path Profiler

 

This repository shares the design flow and architectural details for DynPath along with the Tool Flow and methodology for test case generation and working with FPGA.
The initial implementation of the instruction decoder and control unit in similar to DynRP (https://ieeexplore.ieee.org/document/9190415/). 

Code logic specific to Path Detection and Identification along with PSPIC is uploaded in this repository. This logic can be interfaced with one's function and loop profiler to generate execution path profiling information in hardware.

C code for a sample exhaustive test case is uploaded, for functional testing of the path profiler. The intention is more on creating exhaustive path structures to validate the path profiler behaviour, than achieving a specific code functionality. Therefore function name/description is not applicable.
