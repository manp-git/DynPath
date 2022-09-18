Block diagram: The path profiler has an instruction decoder and a control logic unit
which takes the run-time trace signals from the microprocessor (Xilinx Microblaze [30])
as input, detects the occurrence of processes (loops and functions) and shares it with rest
of the Function and Loop CAM to uniquely identify each. In this process the CAMs
generate a unique tag for each process that is detected. Our implementation supports
unique identification for maximum of 32 processes (16 loops and 16 functions). We
further extend on the usage of these tag bits and include these as primary inputs for
our Path Tracking Unit along with other control signals. Figure FC5.7 shows the block
level implementation diagram of the path profiler. This mainly includes the Instruction
Decoder and Control Unit, the Tracking Unit, the Process Content Addressable Memories, the Path Invocation Count Storage, and the Path specific Process Invocation Count
Storage. This helps in obtaining a very detailed path profiling information. Path Detection and Identification Logic (PDIL) consists of Path Tracking Unit, the Link Circuitry,
and the Path Content Addressable Memory (Path CAM) with a unidirectional flow of
75
information. There is a separate Finite State Machine (FSM) acting as the control unit
to synchronize these 3 modules.

![image](https://user-images.githubusercontent.com/82860513/190897180-eddb8c3b-21d9-4aae-9d0f-51ccfdae5442.png)

Input from the
instruction
decoder, control
logic and
Process CAMs
of the profiler
Paths detected
and shared
once they
Path switch or exit
Tracking
Unit
Link
Circuitry
Path Content
Addressable
Memory
Pass on the
paths one by
one to the Path
CAM for
generating
Path tags
Figure FC5.8: Flow inside Path Detection and Identification Logic
Path Detection and Identification Logic: Figure FC5.8 shows three major blocks present
in the Path Detection and Identification Logic (PDIL), their one-way connectivity and
flow of information. The sub-sections below describe in detail the design choices that
went into the functionality of each block. The Finite State Machine (FSM) shown in
Figure FC5.9 acts as the control unit to synchronize the flow of information between the
above 3 modules, viz. the Path Tracking Unit, the Link Circuitry, and the Path Content
Addressable Memory (Path CAM) needed for the path detect and identification logic in
the proposed path profiler.
Each of the three modules consist of multiple pointers to aid execution of different
operations needed for their functionality. These pointers are reset/initialized in state
S0 of the FSM. S0 is also the reset state of the FSM. In state S1 the FSM awaits the
path detection trigger from the Path Tracking Unit. This aspect will be explained in
greater detail in Sections- 5.4.1 and 5.4.2. In states S2 and S3 of the FSM, the path
matching process of paths already written into the Path CAM are implemented. The
path matching process will be further explained in Section- 5.4.3. As a final step, newly
detected paths are written into the Path CAM in states S4 and S5 respectively of the
FSM, details of which are described later