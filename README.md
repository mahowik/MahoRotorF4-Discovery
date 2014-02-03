MahoRotorF4-Discovery rev 0.1 
=======
Hey guys! 

Here is port of BaseFlight-20131126 (with MultiWii 2.3 features) for popular and cheap but powerful STM32F4-DISCOVERY board + GY-86 sensors board (mpu6050 + hmc5883 + ms5611).
Project was created for CooCox CoIDE 1.7.5 (with toolchain arm-none-eabi-gcc 4_7-2013q3) to simplify stm32 diving for newcomers ;) It's free powerful IDE with user-friendly GUI, debug out from the box etc.

http://www.youtube.com/watch?v=1FZdLTwO87M

Current state (tested in hands):
 - GY-86 connected I2C2: SCL=>PB10, SDA=>PB11
 - 8-channel CPPM (PPM SUM) RC input: PB8
 - USART1 for telemetry/GUI: TX=>PB6, RX=>PB7
 - USART3 for GPS: TX=>PD8, RX=>PD9
 - 4 PWM 400hz outputs (e.g. for X-copter), tested with oscilloscope: PE14, PE13, PE11, PE9. Of course much more PWM outputs possible but for now was tested only these set.
 - Status LEDs:
   Green: state of calibrations, ARM/DISARM etc.
   Orange: HORIZON or ANGLE modes 
   Blue: state of GPS
   

Big thanks to SergDoc for full set of drivers related to STM32F4 & BaseFlight https://github.com/SergDoc/Nev_MultirotorControl

Also thanks for the projects:
- AQ32
- FF32
- MWArmF4
- BaseFlight & MultiWii by default of course :)    
   
Enjoy but remember it's not tested in-flight yet! ;)


Anyway I spent a lot of own free time for this project, so if it's useful for you feel free to press button below :) 

[![Donate](https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=NQ6D8YEWUV88S)


     
  
 



 