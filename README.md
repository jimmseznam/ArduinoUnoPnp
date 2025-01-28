# ArduinoUnoPnp
Control Pick & Place machine (originally 3018 CNC Router Machine for GRBL, about $150) via serial ASCII msg.
Characteristics:
1. Four-axis control (steper drivers x, y, z, angle)
2. PWM DC motor control for vaccu pump
3. Vacuum relay control
4. 

ArduinoUnoPnp.ino is simple code for controling 4 stepper motors (in Timer1 interrupt routine ISR(TIMER1_COMPA_vect)
Parent application PickPlace.exe (Delphi), which controls PnP, sends ASCII machine control messages to Arduino via RS232 from PC. 
Positions messages are received thru interrupt routine ISR(USART_RX_vect). 
Example of ASCII control msg: <G|X|Y|Z|A|Pump|Relay|SpeedDiv>
Where: 


      1. G .. Go
      2. X,Y,X .. x,y,z coordinate [mm]
      3. A     .. Place angle [deg]
      4. Pump  .. 0 .. 255 ~ 0 ..100% PWM control vacuum pump DC motor
      5. Relay .. Air relay 0 - Off, 1 - On 
      6. SpeedDiv .. 0 ~ zero speed, 1 ~ max speed, 2 ~ 1/2 max speed, 3 ~ 1/3 max speed,...
