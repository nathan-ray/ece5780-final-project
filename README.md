# Water Pump Gimbal Final Project Repository
Repository Name: ece5780-final-project <br>
Class: ECE 5780 Spring 2024 <br>
Team: Austin LaMontagne, Hank Thompson, Jaxon Parker, Nathaniel Ray Raharjo. <br>
### Summary
This repository contains the code to operate our water control system. We use a STM32F0 microcontroller with pinouts to control a pump, servo gimbal, and water level sensor within a tank. The pump is controlled by a digital signal sent to a L298 H-Bridge that powers the pump with 12 volts. This pump can either spray water or not based on a flag that is controlled by the water level sensor. When the water level gets below the sensor, a interrupt is triggered to disable the pump. The servo gimbals are controlled using PWM that control the direction and rotation of servo motors connected to the water pump with a tube. The controls reach the STM through UART communication that comes directly from our Raspberry Pi. <br>

Concepts Used: Interrupts, PWM, and UART.

Our demonstration video explains how the project works as well as shows the hardware we used. We also talk about some of the challenges we faced: [Demonstration Video](https://drive.google.com/file/d/1V1j9mKNC-Pv2_Mu0BdX_NTD0BwQLpbGU/view?usp=drive_link/). <br>
### Installation
This repository contains the code with the build environment. We use uVision5 IDE to build and push it to the STM32F0 board. <br>
Utilizes UART communication from a Raspberry Pi to control the servo and pump. The code for our GUI and Raspberry Pi is within our senior project repository: [ECE-Senior-Project](https://github.com/jaxonparker18/ECE-Senior-Project/).<br>
