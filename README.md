# TesLorean-Thermal-Controller
Arduino code to monitor drive and charging temps and control pumps and values

The thermal controller is similar to the module purpose in the Tesla Model S.  It listens for temperature information (via CAN) from around the rest of the car (drive unit, charger, battery) and modifies the operation of pumps, values, coolant heater, and chiller to keep coolant temperatures within operating limits.

==== TESLA COOLANT PUMPS=====================

The coolant pumps out of the Tesla (2015 Model S 70D) are VariMax Intercooler Pumps. Listed for "C4 Corvette 1985-1996" by Lingenfelter Engineering.

They have four control lines...
+12v and GND, PWM and Signal.
PWM is a 5v, 2Hz signal (2 cycles per second)
(switching on for 0.25 secs, and off for 0.25 secs will set speed to 50%)
Signal is PWM-like and indicates the pump speed. **I controlled it with a simple 5v digital pin on an Arduino.

- Target flow rate 720 LPH @ 70 kPa 
- Inlet / Outlet connection: 19 MM Barb 
- Motor syle: Brushless 
- Operating voltage: 8-16 VDC 
- Maximum amp draw: 7.3 Amp with RSDS Software"

It has soft start which means that on applying 12v or adjusting the PWM signal it slowly speeds up or slows down as necessary.

====TESLA COOLANT VALVES==================

The Tesla has a number of coolant diverter valves, one of which is 4 way : TMN 6007370-00-B (two ins, switched between two outs), and others are 3 way : TMN 6007384-00-B (one in, switching between two outs).

The 4-port valve is used to connect the coolant flow into 1 continuous large loop or separate it into 2 smaller loops. Useful for heating/cooling just the battery unit - distinct from whatever the temp of the driveunit. The 3-oprt is used to bypass the radiator and another to bypass the coolant chiller.

The valves are listed as PWM controlled, but this is not correct. There are four wires, +12v and Gnd, Signal and Control. Rather than being PWM, Control should switch between Ground and +12v to switch flow from one outlet to the other (for both the 3 and 4 port valves). Signal indicates value position - but need not be connected for valve control purposes.

3-Port
TMN 6007384-00-B
Electrical Actuated 3/4" 3-port
TMN G9361-0R010
H42M-9000-000

4-Port
TMN 6007370-00-B
Electrical actuated 3/4" 4-port
Invensys 15B16
Motor actuator (12v)
127-00033-001 (90)
H42M-8000-000

