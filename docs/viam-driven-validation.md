# Viam RDK validation drive

The goal of this project is validate the servo controller module in viam this document outline a set of tests that make the motor move, assume the shaft is clear at all time. We will test different behavior of the module (reconfigure etc...) and run the tests each time confirming we get the expected outcome.

Start by adding all the missing features, make sure motor mode switch is supported. 

As you find bugs make sure the are properly described before fixing them 

Read the doc folder to get infos about the state on repo. Confirm any assumptions by looking at the code. 

Before running the test sequence, ask me any remaining questions or clarifications

You may use sudo to get the appropriate privileges

I also want to make sure our module doesn't consume too much cpu (sleep between ticks)



## Tests

### Viam Machine
part_id: 59e04139-bc28-498a-b55a-485f8ea4789c
addr: ethercat-test-main.03jg17i9wj.viam.cloud
loc: 03jg17i9wj

Use the viam cli to change the config of the machine

You will need to run the machine  yourself viam executable is at /home/viam/rdk/bin/Linux-x86_64/viam-server-static and config is at /home/viam/rdk/viam.json

### Python Sample code
Use viam python sdk to connect to the robot and control the motor. I am adding a sample code containing the api keys and all that you will use to connect to it. Change this code to conform to the test sequence outlined in the next sections
``` python
import asyncio

from viam.robot.client import RobotClient
from viam.components.motor import Motor
from viam.services.generic import Generic as GenericService

async def connect():
    opts = RobotClient.Options.with_api_key(
         
        api_key='g6o8sr4zh49yefpuhvpn909vyc0e0ggi',
        
        api_key_id='403431c2-0187-4759-a0db-ab6fc43bd1af'
    )
    
    return await RobotClient.at_address('ethercat-test-main.03jg17i9wj.viam.cloud', opts)

async def main():
    async with await connect() as machine:
        print('Resources:')
        print(machine.resource_names)
        
        # servo
        servo = Motor.from_robot(machine, "servo")
        servo_return_value = await servo.is_moving()
        print(f"servo is_moving return value: {servo_return_value}")

if __name__ == '__main__':
    asyncio.run(main())
```

### Test1: Connect
- Connect to the robot
- List resources
- confirm the servo is in the resources
if this test fails no need to continue
### Test2 SetRPM:
1) Set rpm to 1000, the position should increase. (wait 1.5 seconds)
2) Set rpm to -1000, the position should decrease (wait 1 seconds)
3) Set rpm to 0 , the motor should stop

### Test3 GoFor:
1) GoFor rpm=1200, revolutions=60 (should get to current_pos+60 ~5s)
2) GoFor rpm=1200, revolutions=-60 (should get to current_pos-60 ~5s)
3) GoFor rpm=-1200, revolutions=-60 (should get to current_pos+60 ~5s)
4) GoFor rpm=1200, revolutions=-60 (should get to current_pos-60 ~5s)

### Test4 GoTo:
1) GoTo rpm=2000, targetposition=0 
2) GoTo rpm=2000, targetposition=-100
3) GoTo rpm=2000, targetposition=100
4) GoTo rpm=2000, targetposition=0 

### Test5 Cancellation + Stop + refecjection:
1) GoTo rpm=2000, targetposition=1000 cancel after 6 seconds (asyncio cancel). The motor should stop 
2) GoTo rpm=2000, targetposition=0 
3) GoTo rpm=2000, targetposition=-1000 stop after 6s (use another thread, confirm the Python GoTo gets an error)
4) GoTo rpm=2000, targetposition=0 
5) GoTo rpm=2000, targetposition=1000 then tryp to call SetRpm=1000 this command should be rejected (call from another thread) then call stop
6) GoTo rpm=2000, targetposition=0 

### Test6 DoCommand
You will need to add a docommand to test :
- ethercat obj 6079h 6079h – DC Link Circuit Voltage (get_motor_voltage)
- ethercat obj 6078h – Current Actual Value (get_motor_current_actual_value)
- ethercat obj 6502h – Supported Drive Mode (get_motor_drive_modes) 

Each docommand should be run outside RT context via SDOs

The test will need to be done this way 
1) call  GoTo rpm=2000, targetposition=2000 
2) call each do command 
3) confirm the motor is still moving
4) stop the motor
5) GoTo rpm=2000, targetposition=0 


## What to test

1) Start with an empty config, then add the servo config + module. Confirm they load and confirm you can connect and run the tests

2) Start with an empty config, then add a wrong servo confid. Confirm an error is reported in the logs 

3) Start with an empty config and try to setup a config that would lead to a dc sync0 error (confirm the drives is brought online - tell me if this isn't possible)

4) Start with an empty config, tehn add a correct onfing, run the test and then reconfigure with a valide config confirm the test still passes

5) start with an empty config, add a valid config run the test and then add a wrong config. Confirme we gte an error, then fix the config issue and confirm the test passes
