MahoRotorF4-Discovery rev 0.1 
=======
Hey guys! Here is fork of BaseFlight-20131126 (with MultiWii 2.3 features) for popular and cheap but powerful STM32F4-DISCOVERY board + GY-86 sensors board (mpu6050 + hmc5883 + ms5611).
Project was created for CooCox CoIDE to simplify stm32 diving for newcomers ;) I.e. user-friendly GUI, debug from the box etc. 

Current state (tested in hands points):
 - GY-86 connected I2C2: SCL=>PB10, SDA=>PB11
 - 8-channel CPPM (PPM SUM) RC input: PB8
 - USART1 for telemetry/GUI: TX=>PB6, RX=>PB7
 - USART3 for GPS: TX=>PD8, RX=>PD9
 - 4 PWM 400hz outputs (e.g. for X-copter), tested with oscilloscope: PE14, PE13, PE11, PE9. Of course much more PWM outputs possible but for now was tested only these set.
 - Status LEDs:
   Green: state of calibrations, ARM/DISARM etc.
   Orange: HORIZON or ANGLE modes 
   Blue: state of GPS

Big thanks to SergDoc for full set of drivers related to stm32f4 & baseflight https://github.com/SergDoc/Nev_MultirotorControl
Also thanks to the projects:
- AQ32
- FF32
- MWArmF4
- BaseFlight & MultiWii by default of cause :)    
   
Enjoy but remember it's not tested in air! ;)

     
  
 



 