#!/usr/bin/env python3
"""
imu_logger.py - Background thread for logging MPU6050 data to CSV.
"""

import threading
import serial
import csv
import time

class Colors:
    RED = '\033[91m'
    RESET = '\033[0m'

class IMULogger(threading.Thread):
    def __init__(self, port='/dev/ttyACM0', baud_rate=9600, filename='imu_log.csv'):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.filename = filename
        self.is_logging = False
        self.ser = None

    def run(self):
        self.is_logging = True
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.ser.flush()
            
            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write CSV Header
                writer.writerow(['timestamp', 'yaw', 'pitch', 'roll', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])
                
                sensor_data = {
                    'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
                    'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0,
                    'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0
                }

                while self.is_logging:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                        
                        try:
                            if line.startswith("GYRO:"):
                                # .split() with no arguments automatically handles any mix of spaces/tabs
                                parts = line.replace("GYRO:", "").split()
                                sensor_data['gyro_x'], sensor_data['gyro_y'], sensor_data['gyro_z'] = map(float, parts)
                                
                            elif line.startswith("YPR:"):
                                # Split the squished line into a YPR half and an ACCEL half
                                ypr_part, accel_part = line.split("ACCEL")
                                
                                # Extract YPR
                                ypr_vals = ypr_part.replace("YPR:", "").split()
                                sensor_data['yaw'], sensor_data['pitch'], sensor_data['roll'] = map(float, ypr_vals)
                                
                                # Extract ACCEL (splitting at the colon to bypass the "(m/s^2):" text)
                                accel_vals = accel_part.split(":")[1].split()
                                sensor_data['accel_x'], sensor_data['accel_y'], sensor_data['accel_z'] = map(float, accel_vals)
                                
                                # Both halves are now parsed! Write the full cycle to the CSV.
                                writer.writerow([
                                    time.time(),
                                    sensor_data['yaw'], sensor_data['pitch'], sensor_data['roll'],
                                    sensor_data['accel_x'], sensor_data['accel_y'], sensor_data['accel_z'],
                                    sensor_data['gyro_x'], sensor_data['gyro_y'], sensor_data['gyro_z']
                                ])
                                
                        except (ValueError, IndexError):
                            # Skips any corrupted half-lines that happen when the USB is first plugged in
                            pass 
                            
        except Exception as e:
            print(f"\n{Colors.RED}IMU Logger failed to start or crashed: {e}{Colors.RESET}")
            
    def stop(self):
        """Safely terminates the logging thread and closes the serial port."""
        self.is_logging = False
        if self.ser and self.ser.is_open:
            self.ser.close()
