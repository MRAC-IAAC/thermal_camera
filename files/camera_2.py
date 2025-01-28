#!/usr/bin/env python  # Specifies the interpreter to run the script.
# -*- coding: utf-8 -*-  # Defines the encoding for the script as UTF-8.

# Importing required libraries
from uvctypes import *  # Provides access to UVC (USB Video Class) types and functions for handling thermal cameras.
import cv2  # OpenCV library for image processing.
import numpy as np  # NumPy library for numerical operations.
import serial  # Library for serial communication with Arduino.
import csv  # Library for writing and reading CSV files.
import time  # Library for time-related operations.
from queue import Queue  # Queue for thread-safe data exchange.
from threading import Thread  # Threading library for concurrent execution.
import os  # Library for interacting with the operating system.

# Serial connection configuration
serial_port = '/dev/ttyUSB0'  # Specifies the serial port connected to the Arduino.
baud_rate = 9600  # Baud rate for serial communication.
output_file = 'sensor_data.csv'  # CSV file to log sensor data.

# Directory for saving screenshots
screenshot_dir = "screenshots"  # Directory name for saving thermal camera screenshots.
if not os.path.exists(screenshot_dir):  # Checks if the directory exists.
    os.makedirs(screenshot_dir)  # Creates the directory if it doesn't exist.

# Global variables
temperature = None  # Stores the temperature value received from Arduino.
humidity = None  # Stores the humidity value received from Arduino.
stop_threads = False  # Flag to stop threads when the program exits.
current_state = "Normal"  # Tracks the current state of the environment.
anomaly_arduino_detected = False  # Tracks anomalies detected by Arduino sensors.
anomaly_camera_detected = False  # Tracks anomalies detected by the camera.

# Open the serial connection
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)  # Establishes a serial connection.
    print(f"Connected to {serial_port}")  # Prints confirmation of the connection.
    time.sleep(2)  # Waits for the connection to stabilize.
except serial.SerialException as e:  # Handles exceptions during connection.
    print(f"Error: {e}")  # Prints the error message.
    exit()  # Exits the program.

# Buffer for thermal data
BUF_SIZE = 2  # Size of the buffer for storing thermal frames.
q = Queue(BUF_SIZE)  # Initializes a queue for storing thermal data frames.

# Callback function for handling thermal camera frames
def py_frame_callback(frame, userptr):
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))  # Extracts thermal data.
    data = np.frombuffer(array_pointer.contents, dtype=np.uint16).reshape(frame.contents.height, frame.contents.width)  # Reshapes the data.
    if frame.contents.data_bytes == (2 * frame.contents.width * frame.contents.height) and not q.full():  # Ensures valid data size.
        q.put(data)  # Puts the data into the queue.

# Converts the frame callback to a C-compatible function pointer
PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

# Converts Kelvin to Celsius
def ktoc(val):
    
    return (val - 27315) / 100.0  # Converts raw Kelvin values to Celsius.

# Applies a thermal colormap to the data
def apply_thermal_color(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)  # Normalizes the data for display.
    np.right_shift(data, 8, data)  # Reduces the bit depth for visualization.
    return cv2.applyColorMap(np.uint8(data), cv2.COLORMAP_JET)  # Applies a color map.

# Adds a color bar to the thermal image
def add_colorbar(img, min_temp, max_temp):
    height, width, _ = img.shape  # Gets the image dimensions.
    gradient = np.linspace(255, 0, height, dtype=np.uint8).reshape((height, 1))  # Creates a vertical gradient.
    colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)  # Applies a color map to the gradient.
    colorbar = cv2.resize(colorbar, (50, height))  # Resizes the color bar.
    img_with_bar = np.hstack((img, colorbar))  # Combines the image with the color bar.
    cv2.putText(img_with_bar, f"{max_temp:.1f}C", (width + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)  # Adds max temp text.
    cv2.putText(img_with_bar, f"{min_temp:.1f}C", (width + 10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)  # Adds min temp text.
    return img_with_bar  # Returns the annotated image.

def find_darkest_contours(data, img_color):
    """
    Identifies the darkest areas in the thermal image and calculates their total area.
    Now detects intensity ranges from green to blue.
    """
    normalized_data = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Define intensity range for green to blue
    lower_bound = 0  
    upper_bound = 50 
    mask = cv2.inRange(normalized_data, lower_bound, upper_bound)  # Create binary mask for the range

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Calculate total area of the contours
    total_area = sum(cv2.contourArea(c) for c in contours if cv2.contourArea(c) > 50)

    # Draw contours on the color image
    for c in contours:
        if cv2.contourArea(c) > 50:  # Filter small contours
            cv2.drawContours(img_color, [c], -1, (0, 255, 0), 2)  # Draw contours in green

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
    Includes ambiance status, a 10x10 grid, and average temperatures displayed at the center of each grid cell.
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
            if not q.empty():
                data = q.get()
                data = cv2.resize(data, (640, 480))
                minVal, maxVal, _, _ = cv2.minMaxLoc(data)
                img = apply_thermal_color(data)
                img_with_contours, total_area = find_darkest_contours(data, img)
                img_with_bar = add_colorbar(img_with_contours, ktoc(minVal), ktoc(maxVal))

                
                # Overlay sensor data
                if temperature and humidity:
                    cv2.putText(img_with_bar, f"Temperature: {temperature}C", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(img_with_bar, f"Humidity: {humidity}%", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Check the total area and display appropriate message
                if total_area < 3000:
                    cv2.putText(img_with_bar, f"Area: {total_area:.0f} px", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                else:
                    cv2.putText(img_with_bar, "No Open Window", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # Display ambiance status in the lower-left corner
                temperature_1 = float(temperature)
                humidity_1 = float(humidity)
                if total_area > 3000 and temperature_1 < 25.0 and humidity_1 < 90.0:
                    cv2.putText(img_with_bar, "Good Ambiance", (10, 450),  # Lower-left corner position
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                elif total_area > 3000 and temperature_1 > 25.0 and humidity_1 > 90.0:
                    cv2.putText(img_with_bar, "Open the window", (10, 450),  # Lower-left corner position
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.putText(img_with_bar, "Window is open", (10, 450),  # Lower-left corner position
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # Display the annotated thermal feed
                cv2.imshow('Thermal Camera', img_with_bar)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_threads = True
                    break
    finally:
        libuvc.uvc_stop_streaming(devh)
        libuvc.uvc_exit(ctx)
        cv2.destroyAllWindows()







# Main execution block
if __name__ == '__main__':
    try:
        arduino_thread = Thread(target=log_arduino_data)  # Creates a thread for logging Arduino data.
        camera_thread = Thread(target=display_camera_feed)  # Creates a thread for displaying camera feed.
        arduino_thread.start()  # Starts the Arduino thread.
        camera_thread.start()  # Starts the camera thread.
        arduino_thread.join()  # Waits for the Arduino thread to finish.
        camera_thread.join()  # Waits for the camera thread to finish.
    except KeyboardInterrupt:  # Handles interruption signals (Ctrl+C).
        print("\nExiting...")  # Prints a message indicating exit.
        stop_threads = True  # Sets the flag to stop threads.
        ser.close()  # Closes the serial connection.