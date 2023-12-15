# Wireless_Control_Of_Prototype_Car
Wireless control of a car via arduino and hc12 for telemetry

keyboard_control.py reads "w a s d" from the keyboard (w-> move forward, s-> move backwards, a-> turn left, d-> turn right). and sends it to the serial monitor of the arduino board connected to the computer. transmitter_control.ino reads these "wasd" inputs from the serial monitor and sends it to another arduino through hc12 module in a character structure form wirelessly. reciever_control.ino receives the data via its hc12 connected module and reads data from the serial monitor. 

If the command is to move forward an L298N module is used to regulate voltage into two dc motors of 12V from the power source(battery back) which rotates the motors in the forward direction while the motors are rotated backward if the command is to move backwards by reversing the polarity of current through an in-built H-bridge in the L298N module. PWM pins allow for speed control (not implemented yet). 

![download](https://github.com/Raman-Saini9/Wireless_Control_Of_Prototype_Car/assets/68729255/50e778f1-45d8-4d69-88fe-10648d87700d)



The turning of the vehicle is controlled by TB6600 microstep motor driver and a Nema stepper motor. The motor driver is set to 2.8A and 16 steps. It applies power to the appropriate step motor winding to produce torque. They precisely divide the current between the motor phases, thus positioning the step motor at smaller increments between full steps.


![download](https://github.com/Raman-Saini9/Wireless_Control_Of_Prototype_Car/assets/68729255/92a38b9e-0cb8-446d-b71a-19b4bab9b073)


All the arduino to microstep driver, hc-12 and L298 connection pins are specified in the code itself. 

![images](https://github.com/Raman-Saini9/Wireless_Control_Of_Prototype_Car/assets/68729255/236e8e72-dda8-439b-a246-1d19e97d6308)
