# Teensy 4.1 based Track Logger v2.x Changelog

## Current Version

Version 2.0.0.1 - Initial Checkpoint

 
## History
Version 2.0.0.2 - 2021-03-12
Update development environment to MCUXpresso IDE v11.3.0 [Build 5222]
                                  MCUXpresso SDK v2.9.2 Manifest 3.8.0
                                  
Version 2.0.0.1 - 2021-02-14
Initial checkpoint that includes
 - Basic initialization of USB CDC VCOM Debug Console
 - Basic initialization of FatFS filesystem for Teensy 4.1 microSD socket
 - Initialization of some Tasks for LED blink, SD card logging, an initial implementation of a monitor that will allow some basic interaction with the logger.
 
## To-Do
 [ ]  Implement the queue and data structures for passing data to the logging Task
 [ ]  Implement the ability to store and restore configurations to the SD card
 [ ]  Implement the CAN task to sniff data on the CAN bus
 [ ]  Implement the Bluetooth task to transmit data to a connect iDash app
 [ ]  Implement the GPS task to capture data from the u-Blox Neo-M9N
 [ ]  Implement the IMU task to capture 3-axis linear accelerations and 3-axis angular accelerations