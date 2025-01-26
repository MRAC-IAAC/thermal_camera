# Thermal Camera with Arduino Integration

This project integrates a thermal camera with an Arduino to capture real-time temperature and humidity data. It overlays sensor readings on thermal images, detects the darkest contours in the image, and saves annotated screenshots along with metadata (e.g., temperature, humidity, and contour area) in a CSV file.

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

## Expected Output

### Real-Time Display
- The thermal camera feed will:
  - Highlight the hottest (red) and coldest (blue) points.
  - Show a color bar indicating the temperature range.
  - Overlay:
    - Temperature (from Arduino).
    - Humidity (from Arduino).
    - Area of the darkest (coldest) region.

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


