# Compass Project
DIY Compass, on activation changes direction to point to external provided GPS coordinate using electromagnetic coil to drive the needle to the right position

# Components
## Tracker
A small raspberry Pi Pico connected to GPS and Lora modules
constantly transmitting its location 

## Compass
Consists of a standard compass, beneath hides a small ESP PBC that controls a coil that can rotate 360 using a Servo motor
pointing to a specified GPS coordinate provided using Bluetooth (From the drone)

## Drone
Home antenna near the Compass mounted to a drone so it can be raised for further distance for Lora receiving the Tracker coordinates
And furthers this data to the Compass
