# Thermal Camera with Arduino Integration

<img width="470" alt="Screenshot 2025-02-01 at 16 20 19" src="https://github.com/user-attachments/assets/5965d23d-e20d-474f-86e2-cdf6b615be19" />

This project combines a thermal camera with Arduino-based environmental sensors to monitor indoor conditions in real time. It detects temperature and humidity anomalies, analyzes thermal images, and provides automated responses to maintain optimal indoor environments. When high temperature (>30°C) and humidity (>90%) are detected, the system prompts the user to open a window. The thermal camera then identifies the open window, measures heat loss speed, and tracks temperature normalization. All data, including sensor readings, detected anomalies, and annotated thermal images, are logged for further analysis, making this an effective solution for indoor climate monitoring and optimization.


---

## Features

### Real-Time Thermal Imaging
- Displays thermal images with a color gradient.
- Detects and highlights the darkest regions in the thermal image.
- Displays minimum and maximum temperature values as a color bar.

### Arduino Sensor Integration
- Captures temperature and humidity data from an Arduino via a serial connection.
- Displays real-time sensor readings on the thermal image.

### Data Logging
- Saves the following data to a CSV file (`sensor_data.csv`):
  - Timestamp
  - Temperature
  - Humidity
  - Saved image filename
  - Area of the darkest contours in pixels.

### Screenshot Saving
- Saves annotated thermal images in the `screenshots` directory.

---

## Repository Python Files Navigation:

#### camera_area.py 
- This project is made as an excercise project that calculates the area of darkest parts (coldes) of the image from Thermal Camera). This file was made to practice the workflow of Thermal Camera and to understand work with openCV to contour desired areas of the image and calculate the area of it

#### camera_2.py 
- This project monitors indoor conditions by detecting normal ambient states (around 24°C, 50% humidity) and identifying anomalies (temperature >30°C, humidity >90%) using Arduino sensors. When an anomaly occurs, it prompts to open a window, and the thermal camera detects the window's open state, analyzes its area, and tracks heat loss speed until conditions return to normal.

## WARING: Both of the files work with the following setup:

## Requirements

### Hardware
- **Pure Thermal 2 Interface Board**
- **FLIR Lepton Camera Module**
- **Computer running Linux** (tested on Ubuntu)
- **DHT11 Sensor** (for temperature and humidity)

### Software
- **Visual Studio Code** (or any text editor)
- **Python 3.x**
  - Required libraries:
    - `libuvc`
    - `opencv-python`
    - `numpy`
    - `pyserial`
    - `cmake`
- **Arduino IDE**
  - Required library:
    - `DHT-sensor-library` (by Adafruit)

---

## Step-by-Step Setup

### Step 0: Connect the Arduino with the DHT11 Sensor
- Connect the DHT11 sensor to the Arduino as follows:
  - **GND** (sensor) -> **GND** (Arduino)
  - **VCC** (sensor) -> **5V** (Arduino)
  - **Signal** (sensor) -> **Pin 2** (Arduino)

### Step 1: Clone This Repository

```bash
# Clone this repository and navigate to the files directory
git clone <your-repo-url>
cd thermal-camera/files
```

### Step 2: Configure Arduino

1. Navigate to the `dht11_sensor.ino` file.
2. Open it in the Arduino IDE.
3. Install the `DHT-sensor-library` by Adafruit via the Arduino Library Manager.
4. Ensure the digital pin in the code matches the pin connected to the sensor.
5. Upload the code to the Arduino.
6. Verify the Arduino is outputting correct data via the Serial Monitor.

If the Arduino outputs incorrect data:
- Double-check the wiring (ensure the signal and power cables are correctly placed).
- Restart the Arduino IDE.

### Step 3: Set Up Python Virtual Environment

```bash
# Navigate to the project directory and create a virtual environment
cd thermal-camera/files
python3 -m venv thermcam
source thermcam/bin/activate
```

### Step 4: Install Python Dependencies

```bash
# Install required Python libraries and system dependencies
sudo apt update
sudo apt install -y build-essential cmake libusb-1.0-0-dev libjpeg-dev python3-pip libgtk2.0-dev libgtk-3-dev
pip install numpy opencv-python pyserial
```

### Step 5: Build and Install `libuvc`

```bash
# Clone the libuvc repository and build it
git clone https://github.com/groupgets/libuvc.git
cd libuvc
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### Step 6: Run the Final Script

Activate the virtual environment and run the script:

```bash
source thermcam/bin/activate
sudo python3 camera_area.py
```

---

## Expected Output for camera_area.py

### Real-Time Display
- The thermal camera feed will:
  - Highlight the hottest (red) and coldest (blue) points.
  - Show a color bar indicating the temperature range.
  - Overlay:
    - Temperature (from Arduino).
    - Humidity (from Arduino).
    - Area of the darkest (coldest) region.

## Expected Output for camera_2.py

### Real-Time Display
- The thermal camera feed will:
  - Highlight the hottest (red) and coldest (blue) regions for easy identification.
  - Show a color bar to indicate the full temperature range.
  - Overlay sensor data, including:
    - Temperature readings from Arduino.
    - Humidity levels from Arduino.
    - Area of the coldest (darkest) region detected by the camera.
- Anomaly Detection & Response:
  - When temperature exceeds 30°C and humidity surpasses 90%, the system prompts: “Please open the window”.
  - The thermal camera detects an open window based on low-temperature regions (dark blue) and measures heat loss speed through the open area.
  - When temperature and humidity return to normal, the system signals: “Normal ambient”.
  - Saved Data (sensor_data.csv)


### Saved Data (sensor_data.csv)
- A sample entry:

| Timestamp           | Temperature (C) | Humidity (%) | Photo Filename            | Contour Area (px) |
|---------------------|-----------------|--------------|--------------------------|-------------------|
| 2025-01-26 15:29:19 | 22.6           | 45.0         | screenshots/thermal_0.png | 25434.0          |

### Saved Screenshots
- Annotated thermal images are saved in the `screenshots/` directory.

---

## Troubleshooting

### Issue: Thermal Camera Not Detected
1. Ensure the camera is properly connected via USB.
2. Verify detection using:

   ```bash
   ls /dev/video*
   ```

   If no device appears, check your USB connection and try reconnecting the camera.

### Issue: Arduino Port Not Found
1. Confirm the Arduino is connected and note the port using:

   ```bash
   ls /dev/ttyUSB*
   ```

2. Update the `serial_port` variable in `camera_area.py` to match the detected port (e.g., `/dev/ttyUSB0`).

### Issue: OpenCV Errors
If you encounter errors such as:
- `The function is not implemented` or
- `Cannot query video position`

Ensure OpenCV is installed via:

```bash
sudo apt update
sudo apt install python3-opencv
```

### General Debugging Tips
- Ensure Python dependencies are installed in the virtual environment.
- Run the script with root privileges (`sudo`) to access the thermal camera.
- Check the Arduino serial output via the Serial Monitor in the Arduino IDE if no sensor data is being received.

---

## Repository Structure

```
thermal-camera/
├── README.md                   # Documentation
├── LICENSE                     # License information
├── files/                      # Main directory for scripts
│   ├── thermcam/               # Virtual environment for the project
│   ├── camera_area.py          # Main Python script
│   ├── sensor_data.csv         # CSV log file (auto-generated)
│   ├── screenshots/            # Directory for saved screenshots
│   └── dht11_sensor/           # Arduino sketch for the DHT11 sensor
```

---

## Credits
- [libuvc](https://github.com/groupgets/libuvc)
- [OpenCV](https://opencv.org/)
- Adafruit [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)

---



<!--  DO NOT REMOVE
-->
## Acknowledgements

- Creation of GitHub template: [Marita Georganta](https://www.linkedin.com/in/marita-georganta/) - Robotic Sensing Expert
- Creation of MRAC-IAAC GitHub Structure: [Huanyu Li](https://www.linkedin.com/in/huanyu-li-457590268/) - Robotic Researcher


