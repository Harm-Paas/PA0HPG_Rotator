PA0HPG ANTENNA ROTATOR CONTROLLER.

Local/Remote antenna rotator controller for Ham-Radio antennas.
Set antenna bearing by local control or by means of remote antenna 
control software via a USB link.

By :     Harm Paas PA0HPG
         Zuidlaren, The Netherlands
e-mail : h.paas@rug.nl

Copyright (C) 2021,2022  Harm Paas, PA0HPG, Zuidlaren, The Netherlands


DESCRIPTION

TV-type antenna rotators like the "Channelmaster" and "Stolle" rotors are able to rotate 
a variety of light-weight HF and UHF radio-amateur antennas.
In most cases the control of these rotators is done by means of a steering knob.
A motor in the control device runs at the same speed as the motor in the rotator 
and positions the antenna bearing readout light on the scale of control device in the same direction.

For several reasons the synchronisation between the antenna position on the scale
becomes misaligned with the real bearing position, mostly because the two motors do not
run at the same speed. 

This microcontroller based rotator controller replaces the control unit of these
type of antenna rotators and offers remote computer control via the Yeasu GS232 rotor control protocol.
The unit offers local 3-button control as well as remote computer control via a 
serial USB line for programs like N1MM, PstRotator, LOG4OM, etc.

The Arduino-MEGA2560 based antenna rotator control unit replaces the original control unit
and offers several enhancements compared with the original control unit.
The control unit not only shows the real antenna bearing on a TFT color display 
but can also be connected to a computer via a USB serial link for remote antenna position 
control and position display from the antenna on the computer.
An azimuthal map projection shows the direct position of the antenna on the world map with 
respect to the location of the antenna.
New azimuthal map projections can be made by using the utilities in the Additional Software
section below.

For remote control the Yaesu GS232 compatible command language has been used.
The real antenna bearing angle is obtained by adding a position indicator to
the motor control unit consisting of a 10 turn potentiometer which is attached
to the gearbox of the unit by means of an extra gear. 
In case the control cable is broken a warning will be displayed on the control unit.
If the antenna becomes misaligned on the mast it is possible to correct this
by changing the deviation with respect to the true North. The value will be stored
in EEPROM with the new setting. In The Netherlands it is favourable to set the 
mechanical endstop of the rotor unit in direction West. Therefore the deviation position
of 270 degrees is the default value.
For manual control three pushbuttons for Clockwise, CounterClockwise
and Stop are used. A short Push will turn the antenna one degree in the 
CW or CCW direction. Pressing the CW or CCW button for more than 0.5 seconds will 
put the antenna in the autorotate mode until the endstop is reached or by hitting the
Stop button before. 
The Stop button will set a new deviation value when it is depressed for more than 4 seconds.
In the program an averaging (EMA) algoritm of the antenna potentiometer angle value is used to 
suppress noise and jitter on the indicated antenna position.

MOTOR CONTROL
-------------
Because the hardware for controlling the left-right AC motor is done
by two relays which turn the rotor at full speed into the left or right direction the position
control can only be done with a so called "bang bang" type motor control.
A bang-bang controller is a feedback system controller that switches abruptly between 
two states. By nature this type of control causes the intended antenna position
may differ from the real position, this to avoid oscillations around
the new setpoint.
In the "dead-band" around the new position will, once arrived, no position
update take place.
The dead-band region is controlled by a predefined constant.

A description of this type of controllers can be found at : 
user.ece.cmu.edu/~koopman/ece348/lectures/22_controls_handouts.pdf
(sheet 14 and following). 

A more elaborate PID type of control described in the article above can be 
implemented with the Arduino AutoPID library. However, PID control can 
only be used when a proportional type of motor control is implemented 
in the controller hardware. For this type of control a DC rotator motor
is much easier to control proportionally than an AC motor type.

PUSHBUTTON CONTROL
------------------
Albert van Dalen's Switch and Button control library has been used 
for pushbutton control. For details see www.avdweb.nl : 
"Arduino switch and button library with Short/Long Press, Double Click and Beep" 
The "Switch" library files have been placed in the source 
directory of the program for convenience.

ADDITIONAL SOFTWARE 
-------------------
Create Azimuthal Equidistant map from QTH locator
 * http://ok2pbq.atesystem.cz/prog/ae_map.php
Map needs to be size resampled in a 180x180 pixel map
and placed in a color bitmap array.
 * png2c : Create c program bitmap from a png image:
https://github.com/oleg-codaio/png2c
 * Color picker : www.barth-dev.de/online/rgb565-color-picker/ 
 * Hardware :
 * 1) Arduino Mega 2560R3
2) HX8537 480x320 pixel 3.2 Inch TFT Display Shield
3) 3 momentary push buttons with LED ring lights.
4) 10 turn 2 KOhm Potentiometer in rotator housing with gear
5) Arduino Dual relay board
6) Rotator transformer from the original rotator controller

