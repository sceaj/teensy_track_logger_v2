# teensy_track_logger_v2
Teensy 4.1 track data logger

## Current Status
Note that this project is in the very early stages of development - pretty much anything here could change 
and many things will change.  This section will be kept up to date with the latest project status and also
see CHANGELOG.md for milestones and/or more information on progress/status.

### Features
This repo holds (or will hold) the new teensy 4.1 version of my track data logger.
Major changes from the origin version are:

 * Move to Teensy 4.1 featuring the NXP Semiconductors i.MX 1062 MCU with Arm® Cortex®-M7 core
   running at up to 600MHz
   https://www.pjrc.com/store/teensy41.html
 * Move to FreeRTOS real-time operating system
 * Continue to support CAN bus data collection
 * Enhance vehicle dynamics data by moving to an MPU-6050 3-axis accelerometer and gyro. 
   https://www.sparkfun.com/products/11028
 * Add real-time bluetooth data that can be displayed by iDash.
   https://www.sparkfun.com/products/125776 or https://www.sparkfun.com/products/12577
   (Note that the pinouts swap tx and rx in the alternate part)
 * Upgrade the GPS to U-Blox NEO-M9N with up to 25Hz update rate
   https://www.sparkfun.com/products/15712 (U.FL) or https://www.sparkfun.com/products/17285 (SMA)
 * Add a serial monitor to troubleshoot and or review and save configurations

### Development Tools
 * MCUXpresso IDE v11.2.0 [Build 4120] [2020-07-09]
 * MCUXpresso SDK v2.8.2 [376 2020-08-19] Manifest: v3.6.0
