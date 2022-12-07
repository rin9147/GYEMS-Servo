# NOTICE
This library is under development.

The README may not be sufficient; if you find something not in README, please view the code.

Use of my library is at your own risk. If you have any concerns, please use the original library.

# GYEMS Servo Motor

This servo is a brushless servo motor, which communicating via RS485. The servo is controlled by sending a data packt according to the desired control mode. For more detail about this product, please check on "RMD-L Servo Motor Manual V1.2 -EN" file. 

This project, I made an Arduino library to communicate with "RMD-L" series. So if you are using an MCU as Arduino or equivalent controller, please feel free to modify and adjust to your application.

![](images/servo.png)

# Hardware
1. GYEMS servo motor
2. M5Stack Core2
3. TTL to RS485 module for converting signal(M5 RS485 module)
4. RS485 to USB for confif the servo on PC before use
5. Battery (7.0 to 30.0 V)