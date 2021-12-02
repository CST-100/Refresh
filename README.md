# Refresh
Software for the Refresh rocket flight computer/datalogger.

This is copy pasted from the top of Refresh.ino. That likely has the most up to date info.
 *  V0.11 - DECEMBER 2021
 *  LAVIE OHANA (@lavie154) / CALIFORNIA MODEL AEROSPACE
 *  
 *  It's okay, I have no idea what this code does either.
 *  
 *  Refresh is a 54mm form-factor flight computer designed for
 *  datalogging on high-power rockets. It's my first attempt at
 *  such a device, and definitely my first attempt at programming
 *  something this complex, so view this code at your own risk!
 *  
 *  Refresh's major subsystems:
 *  MCU   - Teensy 4.1
 *  IMU   - Bosch BMI088
 *  BARO  - Bosch BMP388
 *  GNSS  - CDtop PA1616S (Adafruit Ultimate GPS V3)
 *  RADIO - HopeRF RFM69HCW @ 915 MHz
 *  
 *  Pyrotechnic deployments and backup altimeter data are provided
 *  by a PerfectFlite Stratologger CF connected via UART.
 *  
 *  FYI - This code is INCOMPLETE! This is largely bashed together
 *  example code and will slowly approach something more flight-worthy
 *  as time goes on. Refresh itself is at about TRL 5 right now.
 *  
 *  If I were you, I wouldn't trust this code to work! Yet...
