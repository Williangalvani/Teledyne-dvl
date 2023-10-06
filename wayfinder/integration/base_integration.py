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
import json
import threading
import os
import signal
import sys
from mavlink2resthelper import GPS_GLOBAL_ORIGIN_ID, Mavlink2RestHelper
LATLON_TO_CM = 1.1131884502145034e5

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
        self.master = None
        self.should_run = True
        self.prevPose = np.zeros(6)
        self.currAtt = np.zeros(3)
        self.AP_offset_us = -dt.datetime.now().timestamp() * 1e6
        self.wayfinderOffset_us = 0
        self.timestamp = 0
        self.reset_counter = 0
        self.host_setup()

    @staticmethod
    def longitude_scale(lat: float):
        """
        from https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L325
        """
        scale = math.cos(math.radians(lat))
        return max(scale, 0.01)

    def lat_lng_to_NE_XY_cm(self, lat: float, lon: float):
        """
        From https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Common/Location.cpp#L206
        """
        x = (lat - self.origin[0]) * LATLON_TO_CM
        y = self.longitude_scale((lat + self.origin[0]) / 2) * LATLON_TO_CM * (lon - self.origin[1])
        return [x, y]

    def has_origin_set(self) -> bool:
        try:
            old_time = self.mav.get_float("/GPS_GLOBAL_ORIGIN/message/time_usec")
            if math.isnan(old_time):
                print("Unable to read current time for GPS_GLOBAL_ORIGIN, using 0")
                old_time = 0
        except Exception as e:
            print(f"Unable to read current time for GPS_GLOBAL_ORIGIN, using 0: {e}")
            old_time = 0

        for attempt in range(5):
            print("Trying to read origin, try # {attempt}")
            self.mav.request_message(GPS_GLOBAL_ORIGIN_ID)
            time.sleep(0.5)  # make this a timeout?
            try:
                new_origin_data = json.loads(self.mav.get("/GPS_GLOBAL_ORIGIN/message"))
                if new_origin_data["time_usec"] != old_time:
                    self.origin = [new_origin_data["latitude"] * 1e-7, new_origin_data["longitude"] * 1e-7]
                    return True
                continue  # try again
            except Exception as e:
                print(e)
                return False
        return False

    def set_gps_origin(self, lat: float, lon: float) -> None:
        """
        Sets the EKF origin to lat, lon
        """
        self.mav.set_gps_origin(lat, lon)
        self.origin = [float(lat), float(lon)]

    def wayfinderDataCallback(self, dataObj: OutputData, *args):
        '''WayFinder Data Callback Function
        
        WayFinder Data Callback - this processed the data from the WayFinder and 
        passes it to the PixHawk autopilot
        
        Arguments:
            dataObj {OutputData} -- WayFinder output data
        '''

        try:
            dataTimestamp_us = dataObj.get_date_time().timestamp() * 1e6
            logFile.write("%9d: " % dataTimestamp_us)

            dataTimestamp_boot_us = int(dataTimestamp_us + self.AP_offset_us)
            logFile.write("%9d, " % dataTimestamp_boot_us)
            self.timestamp = dataTimestamp_boot_us

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
            vels =  np.matmul(self.rotMatrix, vels)

            deltaTime_us = int(dataTimestamp_boot_us - self.prevTime)
            if self.prevTime == 0:
                deltaTime_us = 0
            logFile.write("%9d, " % deltaTime_us)

            if deltaTime_us < 0:
                logFile.write("\n")
                return

            angle_delta = self.currAtt - self.prevPose[0:3]
            logFile.write("%9.3f %9.3f %9.3f, " % (angle_delta[0], angle_delta[1], \
                angle_delta[2]))
            position_delta = vels * deltaTime_us * 1e-6
            logFile.write("%9.3f %9.3f %9.3f, " % (vels[0], vels[1], vels[2]))

            vel_err_max = 1
            vel_err_min = 0.001
            pct_err_vel = 0.005
            vel_mean = np.mean(vels)
            confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

            self.mav.send_vision(position_delta, angle_delta, dt=deltaTime_us, confidence=confidence)
            self.status = "message sent at %d" % dataTimestamp_boot_us

            self.master.mav.vision_position_delta_send(dataTimestamp_boot_us, 
                deltaTime_us, angle_delta, position_delta, confidence)
            print("%9.3f %9.3f %9.3f | %9.3f | %9.3f %9.3f %9.3f %9.3f | %s" % \
                (vels[0], vels[1], vels[2], dataObj.vel_err, dataObj.range_beam1, \
                    dataObj.range_beam2, dataObj.range_beam3, dataObj.range_beam4, \
                    dataObj.bit_code))
            self.prevTime = dataTimestamp_boot_us
            self.prevPose[0:3] = self.currAtt
            self.data.append([dataObj, self.AP_offset_us, self.currAtt, self.prevPose])
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
        self.master = mavutil.mavlink_connection('tcp:host.docker.internal:14567')
        self.master.wait_heartbeat()


    def set_current_position(self, lat: float, lon: float):
        """
        Sets the EKF origin to lat, lon
        """
        # If origin has never been set, set it
        if not self.has_origin_set():
            print("Origin was never set, trying to set it.")
            self.set_gps_origin(lat, lon)
        else:
            print("Origin has already been set, sending POSITION_ESTIMATE instead")
            # if we already have an origin set, send a new position instead
            x, y = self.lat_lng_to_NE_XY_cm(lat, lon)
            depth = float(self.mav.get("/VFR_HUD/message/alt"))

            attitude = json.loads(self.mav.get("/ATTITUDE/message"))
            attitudes = [attitude["roll"], attitude["pitch"], attitude["yaw"]]
            positions = [x, y, -depth]
            self.reset_counter += 1
            self.mav.send_vision_position_estimate(
                self.timestamp, positions, attitudes, reset_counter=self.reset_counter
            )

    def main_loop(self):
        """Main execution loop
        
        This handles grabbing the rotational velocity data from the ArduPilot to 
        feed back in for making visual odometry work.
        """
        global AP_Timestamp_us

        print("Started loop")
        while True:
            if not self.should_run:
                time.sleep(0.5)
            message = self.master.recv_match(type=['ATTITUDE', "GLOBAL_POSITION_INT"],
                blocking=True, timeout=30)
            if not message:
                continue
            else:
                messageTimestamp_us = dt.datetime.now().timestamp() * 1e6
                AP_Timestamp_us = message.time_boot_ms * 1e3
                self.AP_offset_us = AP_Timestamp_us - messageTimestamp_us
                if message.get_type() == 'ATTITUDE':
                    messageData = message.to_dict()
                    self.currAtt[0] = messageData['roll']
                    self.currAtt[1] = messageData['pitch']
                    self.currAtt[2] = messageData['yaw']
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

        yaw = self.yaw
        pitch = self.pitch
        roll = self.roll
        self.rotMatrix = np.array(\
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



        while not sensor.connect(self.wayFinderPort, 115200):
            print(f"Failed to connect to WayFinder at {self.wayFinderPort}")
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
        sensor.register_ondata_callback(self. wayfinderDataCallback, None)

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
    
    def run(self):
        self.enable()


if __name__ == '__main__':
    driver = WayFinderDriver()
    driver.enable()