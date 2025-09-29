# RoMeLa motor driver gen2
 Motor driver PCB and firmware designed for RoMeLa actuators. 
 

 # PCB reference

<img src="/docs/driver_gen2.jpg" width="400">

[Link to gen2 schematics](/pcb/rev1/schematics_Motor_Gen2.pdf)

<img src="/docs/driver_gen2_U180.jpg" width="400">

[Link to gen2L schematics](/pcb/split/gen2L/schematics_Motor_Gen2L.pdf)

[Link to gen2P schematics](/pcb/split/gen2P/schematics_Motor_Gen2P.pdf)

# Change Log
12/1/2022 Ver. 581
- First version using gcc(STM32CubeIDE)

04/13/2022 Ver. 48D
- Added 0.5ms de-glitch time for E-Stop input and 1sec minimum E-Stop time.

02/09/2022 Ver. 449
- Fixed MCU lock up with RS-485 over run present in Ver. 2BB.

09/27/2021 Ver. 33B
- Random ABS Posi error and Init error on power-up.

05/27/2021 Ver. 2BB
- Fixed cogging torque comp. issue.

03/31/2021 Ver. 27F
- Limit Id Max changed to Limit Acc Max
- Limit Iq Max changed to Limit I Max
- Added external Estop signal (wiring change) and command
- Acceleration limited S curve added to Mode 2 (position mode).
- Goal velocity and Iq feedforward term enabled for Mode 3. (Allow Id, Iq, Velocity, Position reference all at once)

09/29/2020 Ver. 130-137
- Added support for KB01_R18 and KB01_R24. (External AS5047 Encoder)

06/23/2020 Ver. D7
- Fixed joint limit issue.

05/20/2020 Ver. B4
- GEN2 Initial release.
