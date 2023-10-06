#!/usr/bin/env python3

import time
from pymavlink import mavutil
from dvl.dvl import Dvl
from dvl.system import OutputData
import math
import datetime as dt
import numpy as np
import traceback
import struct
import threading
import os
import signal
import sys
from mavlink2resthelper import GPS_GLOBAL_ORIGIN_ID, Mavlink2RestHelper

class WayFinderDriver(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.wayFinderPort = "/dev/ttyUSB0"
        self.roll = np.radians(180)
        self.pitch = np.radians(0)
        self.yaw = np.radians(135)
        self.prevTime = 0
        self.data = []
        self.status = "starting..."
        self.mav = Mavlink2RestHelper()
        self.should_run = True
        self.prevPose = np.zeros(6)
        self.currAtt = np.zeros(3)
        self.AP_offset_us = -dt.datetime.now().timestamp() * 1e6
        self.wayfinderOffset_us = 0

    def wayfinderDataCallback(self, dataObj: OutputData, *args):
        '''WayFinder Data Callback Function
        
        WayFinder Data Callback - this processed the data from the WayFinder and 
        passes it to the PixHawk autopilot
        
        Arguments:
            dataObj {OutputData} -- WayFinder output data
        '''
        global master
        global wayfinderOffset_us
        global AP_offset_us
        global prevPose
        global currAtt
        global prevTime
        global sensor
        global logFile
        global rotMatrix
        global mav

        try:
            dataTimestamp_us = dataObj.get_date_time().timestamp() * 1e6
            logFile.write("%9d: " % dataTimestamp_us)

            dataTimestamp_boot_us = int(dataTimestamp_us + AP_offset_us)
            logFile.write("%9d, " % dataTimestamp_boot_us)

            if math.isnan(dataObj.vel_x) or math.isnan(dataObj.vel_y) or \
                    math.isnan(dataObj.vel_z) or math.isnan(dataObj.vel_err):
                logFile.write("NaN velocities\n")
                return

            if dataObj.is_velocity_valid():
                vels = np.array([dataObj.vel_x, dataObj.vel_y, dataObj.vel_z])   # m/s
                logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))
            else:
                logFile.write("Invalid velocities\n")
                return
            vels =  np.matmul(rotMatrix, vels)

            deltaTime_us = int(dataTimestamp_boot_us - prevTime)
            if prevTime == 0:
                deltaTime_us = 0
            logFile.write("%9d, " % deltaTime_us)

            if deltaTime_us < 0:
                logFile.write("\n")
                return

            angle_delta = currAtt - prevPose[0:3]
            logFile.write("%9.3f %9.3f %9.3f, " % (angle_delta[0], angle_delta[1], \
                angle_delta[2]))
            position_delta = vels * deltaTime_us * 1e-6
            logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))

            vel_err_max = 1
            vel_err_min = 0.001
            pct_err_vel = 0.005
            vel_mean = np.mean(vels)
            confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

            mav.send_vision(position_delta, angle_delta, dt=deltaTime_us, confidence=confidence)
            self.status = "message sent at %d" % dataTimestamp_boot_us

            master.mav.vision_position_delta_send(dataTimestamp_boot_us, 
                deltaTime_us, angle_delta, position_delta, confidence)
            print("%9.3f %9.3f %9.3f | %9.3f | %9.3f %9.3f %9.3f %9.3f | %s" % \
                (vels[0], vels[1], vels[2], dataObj.vel_err, dataObj.range_beam1, \
                    dataObj.range_beam2, dataObj.range_beam3, dataObj.range_beam4, \
                    dataObj.bit_code))
            prevTime = dataTimestamp_boot_us
            prevPose[0:3] = currAtt
            self.data.append([dataObj, AP_offset_us, currAtt, prevPose])
            logFile.write("\n")
        except Exception as e:
            print(e)
            traceback.print_exc()
            logFile.write("EXCEPTION\n")

        logFile.flush()

    def signal_handler(self, *args, **kwargs):
        self.sensor.disconnect()
        sys.exit(0)

    def host_setup(self):
        """Sets up the host system interface
        
        In this case, we connect to the Ardupilot on TCP port 14567 on localhost.
        This is the MAVProxy instance that is running on the companion computer.
        """
        global master
        master = mavutil.mavlink_connection('tcp:host.docker.internal:14567')
        master.wait_heartbeat()


    def main_loop(self):
        """Main execution loop
        
        This handles grabbing the rotational velocity data from the ArduPilot to 
        feed back in for making visual odometry work.
        """
        global master
        global AP_offset_us
        global AP_Timestamp_us
        global currAtt

        print("Started loop")
        while True:
            if not self.should_run:
                time.sleep(0.5)
            message = master.recv_match(type=['ATTITUDE', "GLOBAL_POSITION_INT"],
                blocking=True, timeout=30)
            if not message:
                continue
            else:
                messageTimestamp_us = dt.datetime.now().timestamp() * 1e6
                AP_Timestamp_us = message.time_boot_ms * 1e3
                AP_offset_us = AP_Timestamp_us - messageTimestamp_us
                if message.get_type() == 'ATTITUDE':
                    messageData = message.to_dict()
                    currAtt[0] = messageData['roll']
                    currAtt[1] = messageData['pitch']
                    currAtt[2] = messageData['yaw']
                else:
                    print("Unknown")

    def log_setup(self):
        """Configures the data logging
        
        """
        global logFile

        lastLogPath = os.path.join("/usr/blueos/extensions/teledyne/", "lastlog.txt")
        try:
            with open(lastLogPath, 'r') as lastLog:
                logNum = int(lastLog.readline()) + 1
        except Exception:
            logNum = 0

        with open(lastLogPath, 'w') as lastLog:
            lastLog.write("%d\n" % logNum)

        logFile = open(os.path.join("/usr/blueos/extensions/teledyne/", "%06d.log" % logNum), 'w')


    def transform_setup(self):
        """Configures the coordinate transform
        
        """
        global rotMatrix
        global yaw, pitch, roll
        rotMatrix = np.array(\
        [
            [np.cos(yaw) * np.cos(pitch), 
            np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
            np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)
            ],
            [np.sin(yaw) * np.cos(pitch), 
            np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
            np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)
            ],
            [-np.sin(pitch),
            np.cos(pitch) * np.sin(roll),
            np.cos(pitch) * np.cos(roll)
            ]
        ])

    def sensor_setup(self):
        """Configures the WayFinder
        
        This also synchronizes the WayFinder and host computer clocks.  We wait for
        the top of the 2nd 1 second interval before sending the set time command.
        """
        global sensor

        sensor = Dvl()



        while not sensor.connect(wayFinderPort, 115200):
            print(f"Failed to connect to WayFinder at {wayFinderPort}")
            time.sleep(1)

        sensor.reset_to_defaults()
        sensor.enter_command_mode()
        twoseconds = dt.datetime.now() + dt.timedelta(seconds=2)
        timetarget = dt.datetime(twoseconds.year, twoseconds.month, twoseconds.day,
            twoseconds.hour, twoseconds.minute, twoseconds.second)
        now = dt.datetime.now()
        time.sleep((timetarget - now).total_seconds())
        
        sensor.set_time(dt.datetime.now())

        sensor.exit_command_mode()
        sensor.register_ondata_callback(wayfinderDataCallback, None)

    def enable(self):
        self.should_run = True
        self.log_setup()
        self.transform_setup()
        self.host_setup()
        self.sensor_setup()
        self.status = "running"
        signal.signal(signal.SIGINT, self.signal_handler)
        self.main_loop()

    def disable(self):
        self.should_run = False
        self.status = "stopped"
        if hasattr(self, 'sensor'):
            self.sensor.disconnect()

    def get_status(self):
        return self.status


if __name__ == '__main__':
    driver = WayFinderDriver()
    driver.enable()