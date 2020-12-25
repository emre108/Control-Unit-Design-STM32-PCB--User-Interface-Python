In this project, all 12V three pump motors and one solenoid valve are controlled 
by trimmer potentiometers and a 5V analog input.
For switching and controlling the velocity of the motors n-channel MOSFETs 
and Half-bridge drivers were implemented.
Two four-layer PCBs were designed. 
One for driving the motors and the other one for the rest of the work. 
All power regulation circuitry, pull up-pull down resistors, voltage dividers, 
RC filters, etc. were implemented on the PCB board.
 A flow sensor was used to calculate how many liters of water going through the tube in an hour.
 There are also four digital inputs for an indication of the process. 
STM32F103 was the main controller of the process. For 12-bit resolution 4 channel analog inputs, DMA peripheral is used.
 For communication with the interface, UART was chosen. 
To generate PWM signals timer1, to capture and calculate the frequency of the flow sensor, timer2 is activated.
 The motors and LEDs can be controlled by a user via an interface program written in Python. 
In this program, the user can see analog values, digital signals, and the frequency of the sensor.
 Baud-rate and com-port settings are also available in the python interface. 
The thread feature was used to listen to a com-port that is connected.
 All the motors(duty cycles, ON/OFF) and indicators can also be controlled by the buttons via Python. 
