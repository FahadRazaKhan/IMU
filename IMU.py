import logging
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from Adafruit_BNO055 import BNO055
from pid_controller.pid import PID

# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
stime = np.array([])
sHeading = np.array([])
pid = PID(p=1, i=0, d=2)
try:
    while True:
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = bno.read_euler()
    
        # Print everything out.
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(
              heading, roll, pitch))
        pid_output = pid(feedback=heading)
        print('Controller Output = {0:0.2F}'.format(pid_output))
        newTime = time.clock()
        stime = np.append(stime,newTime)
        sHeading = np.append(sHeading,heading)
        time.sleep(1)

except KeyboardInterrupt:
    plt.plot(stime,sHeading,'ro')
    print('Plotting Graph...')
    time.sleep(1)
    plt.show()
