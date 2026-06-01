# Ethercat master VIAM module 
Your objective is to create a VIAM module capable of acting as a Ethercat master module, the ethercat master implementation should a library that will not be a module and a generic-servo driver which will allow controlling generic servos over ethercat. The last bit will be the a VIAM module with a complete viam API and configuration support 
Both items will be implemented in the same repository for now (might be split later) and will be written in c++20 using boost, cmake, and clang-19 for tidy and format 

## VIAM ethercat master library
The library will be built on top of SOEM
### Configuration
- target_loop_rate_hz: (max 1kHz)
- ifname: interface to bind
These parameters can be added in the genereic servo driver for now 
### Reference design
- /home/viam/ethercat/reference-src/universal-robots and /home/viam/ethercat/reference-src/yaskawa-robots contain examples of a state machine opatter that should be the basis for the CiA402 State Machine
- /home/viam/ethercat/reference-src/fastcat contains a reference application for a ethercat master servo driver. It uses https://github.com/openethercatsociety/soem with will be use by the ethercat master lib for the VIAM implementation
### Configuring RPDOS/TPDOS
The library should offer the ability for a driver top configure PDOS and TPDOS as they see fit. It is on the driver writer to confirm the controller can be configured in such way. Howerver the library should surface clear errros that helps the driver code programmer to understand what went wrong.
Some manufacturer provide default PDOs, the librays should no need to explicitly parse them, but is should provide an API drive programmers can use to retreive raw data from a PDOS
The PDOs shoud have an interface that make is easy to consume of construct the state
for example rpdos.get<u16>() return a u16 integet IF there enough bytes to read, if will then move the cursor consuming two bytes of the inner buffer. Simillarly tpdos.write<u16> shoud write a u16 in the inner buffer and update the cursor internally. 
### CiA402 State Machine
The driver should provide an skeleton of impletation for a state machine. Note that the library doesn't drive the staTE MACHINE the driver (consumer of library) is responsible for this, hence it should be in effect extensible. There is an option to drive it by default through SDO but a user should be able to get the raw control work and feed the status word using PDOs and TPDOs
### Error reporting
errors should be written in clear text and be easily understanable (exception)
### RPDO and TPDO caching
The SOEM should always cache the last RPDOs received which can be retreived and any time by the driver, retrival should not endager the realtime constrains of ethercat. TPDOs should be cached until they are written on the bus (after that they became empty) you can re-write a TPDO if it wasn't sent yet.
The cach should be smartly built and resilient but also simple

## Generic servo driver module
The viam module to controller cgeneric servo driver will first target and A6 stepperoline ethercat driver. Note that we will keep it generic so no reference toward the underlying motor controller should be made. The manual can be found at /home/viam/ethercat/a6-manuals/A6-EC_series_servo_drive_manual.pdf
### Configuration of the generic servo driver
- control_type (for not PP and PV modes are supported only) we should be able to reference these mode using "PP" or "PV"
- max_motor_speed_rpm: the maximum motor speed in RPM (cannot be nedagtive)
- peak_current_limit_amps: the maximum peak current applyaibe in amps (float)
- gear_ratio: the gear ration (float)
- counts_per_rev: counts per revolution

### Control mode
Approriatley control the motor according to the configuration.
Use the only configurable PDOS to configuring for either mode supported. Alsways report speed and current position

## Build, CI, and Lint
Use  /home/viam/ethercat/reference-src/universal-robots as best in class for CI and linting, install clang-19 if needed. The docker image should follow the same principle but be adapted to the contrain of the driver. 
### Conan
Connan will be use for building artifacts but is outside of the scope of work now.

### Testing
Simple, clear and small tests shdould be written will going through dev.

### Code writing
Keep the code focus on clarity.simplicity. modularity and composability. No fucntions should exceed 200LOC except in specific case.
### Commits
Regurlary commit your work


## Team

Build a team using teamcreate. Act as a coordinator 

### Librarian 
Keep the knowledge of the reference design and manuals in memory, andswer questions from other teammeate
### Devil Advocate
Sounding board of the team, plan and decision should be ran by him
### C++ expert
Wroite the c++ code
### Code Architect
Design high level code and provide clear guidance on how to test the designs and code



