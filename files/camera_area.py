#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uvctypes import *
import cv2
import numpy as np
import serial
import csv
import time
from queue import Queue
from threading import Thread
import os

# Serial connection configuration
# Configuring the serial port and baud rate for communication with the Arduino
serial_port = '/dev/ttyUSB0'
baud_rate = 9600
output_file = 'sensor_data.csv'

# Directory for saving screenshots
# Create a "screenshots" directory if it doesn't already exist
screenshot_dir = "screenshots"
if not os.path.exists(screenshot_dir):
    os.makedirs(screenshot_dir)

# Global variables for sensor data and thread control
temperature = None  # Variable to store temperature data from the Arduino
humidity = None  # Variable to store humidity data from the Arduino
stop_threads = False  # Boolean flag to stop threads gracefully

# Open the serial connection
# Establish communication with the Arduino
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Connected to {serial_port}")
    time.sleep(2)  # Allow the connection to stabilize
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()

# Buffer for thermal data
BUF_SIZE = 2  # Size of the queue buffer for storing thermal data frames
q = Queue(BUF_SIZE)

def py_frame_callback(frame, userptr):
    """
    Callback to handle incoming thermal frames.
    Converts the raw frame data into a format that can be processed.
    """
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    data = np.frombuffer(array_pointer.contents, dtype=np.uint16).reshape(frame.contents.height, frame.contents.width)
    if frame.contents.data_bytes == (2 * frame.contents.width * frame.contents.height) and not q.full():
        q.put(data)  # Add the processed frame to the queue

PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

def ktoc(val):
    """
    Converts a temperature value from Kelvin to Celsius.
    """
    return (val - 27315) / 100.0

def apply_thermal_color(data):
    """
    Generates a thermal image with a colormap.
    """
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.applyColorMap(np.uint8(data), cv2.COLORMAP_JET)

def add_colorbar(img, min_temp, max_temp):
    """
    Adds a color bar to the right side of the thermal image to indicate temperature ranges.
    """
    height, width, _ = img.shape
    gradient = np.linspace(255, 0, height, dtype=np.uint8).reshape((height, 1))
    colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)
    colorbar = cv2.resize(colorbar, (50, height))
    img_with_bar = np.hstack((img, colorbar))
    cv2.putText(img_with_bar, f"{max_temp:.1f}C", (width + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(img_with_bar, f"{min_temp:.1f}C", (width + 10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img_with_bar

def find_darkest_contours(data, img_color):
    """
    Identifies the darkest areas in the thermal image and calculates their total area.
    """
    normalized_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, thresh = cv2.threshold(normalized_data, 50, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total_area = sum(cv2.contourArea(c) for c in contours if cv2.contourArea(c) > 50)
    for c in contours:
        if cv2.contourArea(c) > 50:
            cv2.drawContours(img_color, [c], -1, (0, 0, 255), 2)
    return img_color, total_area

def log_arduino_data():
    """
    Logs temperature and humidity data from Arduino to a CSV file and saves annotated thermal images.
    """
    global temperature, humidity, stop_threads
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Temperature (C)", "Humidity (%)", "Photo Filename", "Contour Area (px)"])
        print("Logging data to CSV file...")

        while not stop_threads:
            if ser.in_waiting > 0:  # Check if Arduino data is available
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "The temperature is:" in line:
                    temperature = ser.readline().decode('utf-8', errors='ignore').strip().split()[0]
                elif "The humidity is:" in line:
                    humidity = ser.readline().decode('utf-8', errors='ignore').strip().split()[0]

                    # Capture a thermal image frame
                    data = q.get(True, 500)
                    if data is not None:
                        data = cv2.resize(data, (640, 480))
                        minVal, maxVal, _, _ = cv2.minMaxLoc(data)
                        img = apply_thermal_color(data)
                        img_with_contours, total_area = find_darkest_contours(data, img)
                        img_with_bar = add_colorbar(img_with_contours, ktoc(minVal), ktoc(maxVal))

                        # Annotate image with sensor data
                        if temperature and humidity:
                            cv2.putText(img_with_bar, f"Temperature: {temperature}C", (10, 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            cv2.putText(img_with_bar, f"Humidity: {humidity}%", (10, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                        cv2.putText(img_with_bar, f"Darkest Area: {total_area:.0f} px", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                        # Save image and log data
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        photo_filename = os.path.join(screenshot_dir, f"thermal_{timestamp}.png")
                        cv2.imwrite(photo_filename, img_with_bar)

                        csv_timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                        writer.writerow([csv_timestamp, temperature, humidity, photo_filename, total_area])
                        print(f"Logged: {csv_timestamp}, {temperature}C, {humidity}%, {photo_filename}, {total_area}px")
                        file.flush()

def display_camera_feed():
    """
    Displays the thermal camera feed with real-time annotations and sensor overlays.
    """
    global temperature, humidity, stop_threads
    ctx, dev, devh, ctrl = POINTER(uvc_context)(), POINTER(uvc_device)(), POINTER(uvc_device_handle)(), uvc_stream_ctrl()

    if libuvc.uvc_init(byref(ctx), 0) < 0:
        print("uvc_init error")
        return
    if libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0) < 0:
        print("uvc_find_device error")
        return
    if libuvc.uvc_open(dev, byref(devh)) < 0:
        print("uvc_open error")
        return

    print("Device opened!")
    frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
    if not frame_formats:
        print("Device does not support Y16")
        return

    libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval))
    if libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0) < 0:
        print("uvc_start_streaming failed")
        return

    try:
        while not stop_threads:
            data = q.get(True, 500)
            if data is None:
                continue
            data = cv2.resize(data, (640, 480))
            minVal, maxVal, _, _ = cv2.minMaxLoc(data)
            img = apply_thermal_color(data)
            img_with_contours, total_area = find_darkest_contours(data, img)
            img_with_bar = add_colorbar(img_with_contours, ktoc(minVal), ktoc(maxVal))

            # Overlay sensor data and contour area
            if temperature and humidity:
                cv2.putText(img_with_bar, f"Temperature: {temperature}C", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(img_with_bar, f"Humidity: {humidity}%", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(img_with_bar, f"Darkest Area: {total_area:.0f} px", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow('Thermal Camera', img_with_bar)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_threads = True
                break
    finally:
        libuvc.uvc_stop_streaming(devh)
        libuvc.uvc_exit(ctx)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    """
    Main program to start threads for logging Arduino data and displaying the thermal feed.
    """
    try:
        arduino_thread = Thread(target=log_arduino_data)
        camera_thread = Thread(target=display_camera_feed)
        arduino_thread.start()
        camera_thread.start()
        arduino_thread.join()
        camera_thread.join()
    except KeyboardInterrupt:
        print("\nExiting...")
        stop_threads = True
        ser.close()