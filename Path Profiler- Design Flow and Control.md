# Path Profiler Circuitry


## Block diagram: 
![image](https://user-images.githubusercontent.com/82860513/192847598-a9ee8868-8c56-4182-a514-51ba90560a07.png)


The path profiler has an instruction decoder and a control logic unit which takes the run-time trace signals from the microprocessor (Xilinx Microblaze) as input, detects the occurrence of processes (loops and functions) and shares it with rest of the Function and Loop CAM to uniquely identify each. In this process the CAMs generate a unique tag for each process that is detected. This is similar to DynRP. Our implementation supports unique identification for maximum of 32 processes (16 loops and 16 functions). We further extend on the usage of these tag bits and include these as primary inputs for our Path Tracking Unit along with other control signals. The figures shows the block level implementation diagram of the path profiler. This mainly includes the Instruction Decoder and Control Unit, the Tracking Unit, the Process Content Addressable Memories, the Path Invocation Count Storage, and the Path specific Process Invocation Count Storage. This helps in obtaining a very detailed path profiling information. Path Detection and Identification Logic (PDIL) consists of Path Tracking Unit, the Link Circuitry, and the Path Content Addressable Memory (Path CAM) with a unidirectional flow of information. There is a separate Finite State Machine (FSM) acting as the control unit to synchronize these 3 modules.
![image](https://user-images.githubusercontent.com/82860513/192847200-3e92d06d-09ae-446c-9e5d-7ec6b2fd27de.png)

## Path Detection and Identification Logic: 
Figure above shows three major blocks present in the Path Detection and Identification Logic (PDIL), their one-way connectivity and flow of information. The Finite State Machine (FSM) shown in Figure below acts as the control unit to synchronize the flow of information between the above 3 modules, viz. the Path Tracking Unit, the Link Circuitry, and the Path Content Addressable Memory (Path CAM) needed for the path detect and identification logic in the proposed path profiler.

Each of the three modules consist of multiple pointers to aid execution of different operations needed for their functionality. These pointers are reset/initialized in state S0 of the FSM. S0 is also the reset state of the FSM. In state S1 the FSM awaits the path detection trigger from the Path Tracking Unit. In states S2 and S3 of the FSM, the path matching process of paths already written into the Path CAM are implemented. As a final step, newly detected paths are written into the Path CAM in states S4 and S5 respectively of the FSM.

![image](https://user-images.githubusercontent.com/82860513/190897180-eddb8c3b-21d9-4aae-9d0f-51ccfdae5442.png)


