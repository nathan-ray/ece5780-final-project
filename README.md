# Water Pump Gimbal Final Project Repository
Repository name: ece5780-final-project <br>
Class: ECE5780 Spring 2024 <br>
### Team 
Austin LaMontagne, Hank Thompson, Jaxon Parker, Nathaniel Ray Raharjo. <br>
### Summary
This repository contains the code to operate our water control system. We use a STM32F0 microcontroller with pinouts to control a pump, servo gimbal, and water level sensor within a tank. The pump is controlled by a digital signal sent to a L298 H-Bridge that powers the pump with 12 volts. This pump can either spray water or not based on a flag that is controlled by the water level sensor. When the water level gets below the sensor, a interrupt is triggered to disable the pump. The servo gimbals are controlled using PWM that control the direction and rotation of servo motors connected to the water pump with a tube. The controls reach the STM through UART communication that comes directly from our raspberry pi. <br>
### Installation
This repository contains the code with the build environment. We use uVision5 IDE to build and push it to the STM32F0 board. <br>
Utilizes UART communication from a raspberry pi to control the servo and pump. The code for our GUI and raspberry pi code are within the repository below. <br>
[ECE-Senior-Project](https://github.com/jaxonparker18/ECE-Senior-Project/).