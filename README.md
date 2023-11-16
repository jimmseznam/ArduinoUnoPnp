# ArduinoUnoPnp
Control PnP machine - 4 steper drivers (4 axis), PWM DC motor (vaccu pump), one air relay via serial ASCII msg
ArduinoUnoPnp.ino is simple code for controling 4 stepper motors (in Timer1 interrupt routine ISR(TIMER1_COMPA_vect)
Positions commands for motion are sent to Arduino via serial ASCII messages from PC, where run parent app. 
