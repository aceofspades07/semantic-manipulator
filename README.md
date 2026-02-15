# [AfA2026-PhysicalAI] Semantic Manipulator - AN NLP-Driven Robotic Arm

Industrial robotic arms are traditionally complex to program, requiring specialized knowledge of coordinate systems, kinematics, and pendant programming. This creates a massive barrier to entry for non-technical users who want to utilize robotics for dynamic tasks like sorting, organization, or lab automation. 
The Semantic Manipulator" is an Embodied AI Agent that bridges the gap between human language and physical actuation. By integrating a Natural Language Processing Model (NLP) with the Arduino UNO Q kit, this project allows users to control a 4-DOF robotic arm using natural language commands (e.g., "Pick up the red block and move it to the right"). 

In this application, we have implemented a robust system to detect and segment coloured Jenga blocks, following which, a user can enter Natural Language prompts to control a Robotic Arm to pick up a block, gently place a block, and immediately drop a block.

https://youtu.be/FhsI4L-pOcw
 
The system utilizes computer vision for detection and colour segmentation of Jenga blocks. It translates unstructured text entered by the user into structured JSON protocols, which the Arduino Uno Q parses to orchestrate a safety-critical Finite State Machine (FSM). 




## Prerequisites

- Arduino UNO Q (Main Controller): The central brain responsible for perception, parsing logic, state management, and safety validation. 
- Python 3.x
- Logitech Webcam provided with the kit or RealSense camera (Intel RealSense D400 series recommended) : The primary vision sensor used for color segmentation and ArUco marker pose estimation. 
- RoArm-M2: A 4-DOF Robotic Arm (ESP32-based) handling the Inverse Kinematics (IK) and servo actuation.


## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/szyfrowac/Qualcomm-AI-Challenge
cd Qualcomm-AI-Challenge
```

### 2. Create Virtual Environment

```bash
python3 -m venv qualcomm
source qualcomm/bin/activate  # On Windows: qualcomm\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure RealSense Libraries

Copy the RealSense libraries to your virtual environment (this is only if the camera being used is an Intel Realsense. If you are using a webcam, you may ignore this):

```bash
cp librealsense qualcomm/lib/python/site-packages/
cp pyrealsense qualcomm/lib/python/site-packages/
```

**Note**: If you encounter errors related to `librealsense`, rename it to `librealsense2.so.2.57`:

```bash
cd qualcomm/lib/python/site-packages/
mv librealsense librealsense2.so.2.57
```

**Another Note** : If you do not have a Realsense camera and wish to use a webcam, you may change the code inside of detect_jenga.py accordingly.  

## Usage

### Testing Object Detection

Before running the main application, test the camera and object detection:

```bash
python detect_jenga.py
```

This script will verify that the RealSense camera is working correctly and can detect Jenga blocks.

### Running the Main Application

#### First-Time Setup

1. **Ensure Internet Connection**: Required for downloading necessary models and dependencies
2. **Connect to Robot Network**: 
   - Connect your computer to the `roarm2` WiFi network
   - Default password (if applicable): [Add password here]

#### Launch the Application

```bash
python homepage.py
```

The Gradio interface will launch in your default web browser, typically at `http://localhost:7860`

### Interacting with the Robot

1. **Start the Camera Feed**: The RealSense camera will begin streaming video
2. **Object Detection**: The system will automatically detect objects in the camera's field of view
3. **Select Operation**: Choose from the available operations:
   - **Pick**: Grasp an object from the detected position
   - **Place**: Position the object at a specified location
   - **Drop**: Release the object at the current position
4. **Execute**: Click the execute button to perform the selected operation

## Project Structure

```
Qualcomm-AI-Challenge/
├── homepage.py              # Main Gradio application
├── detect_jenga.py          # Object detection testing script
├── requirements.txt         # Python dependencies
├── librealsense            # RealSense library
├── pyrealsense             # Python RealSense bindings
├── models/                 # Pre-trained models (if applicable)
├── utils/                  # Utility functions
└── README.md              # This file
```

## Troubleshooting

### Common Issues

1. **RealSense Library Error**
   ```
   Error: librealsense not found
   ```
   **Solution**: Rename the library file to `librealsense2.so.2.57` as described in the installation steps.

2. **Cannot Connect to Robot**
   ```
   Error: Connection to roarm2 failed
   ```
   **Solution**: Ensure you are connected to the `roarm2` WiFi network and the robot is powered on.

3. **Import Errors on First Run**
   ```
   Error: Module not found
   ```
   **Solution**: Ensure you have an active internet connection during the first run to download required packages.

4. **Camera Not Detected**
   ```
   Error: No RealSense device detected
   ```
   **Solution**: 
   - Check that the RealSense camera is properly connected via USB 3.0
   - Run `detect_jenga.py` to verify camera functionality
   - Try reconnecting the camera or using a different USB port

## Technical Details

### Hardware Requirements

- **Robot**: RoArm-M2 robotic arm
- **Camera**: Intel RealSense D400 series
- **Arduino UNO Q**:
  - USB 3.0 port
  - WiFi capability
  - Python 3.7 or higher

### Software Components

- **Gradio**: Web-based user interface
- **RealSense SDK**: Camera control and depth sensing
- **OpenCV**: Image processing
- **NumPy**: Numerical computations
- **RoArm-M2 SDK**: Robot control

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

This project is licensed under Mozilla Public License 2.0

## Contact

For questions or support, please open an issue on the [GitHub repository](https://github.com/szyfrowac/Qualcomm-AI-Challenge).

## Future Enhancements

- [ ] Multi-object detection and tracking
- [ ] Voice command integration
- [ ] Mobile app control
- [ ] Advanced gripper control for various object shapes
- [ ] Real-time performance analytics

---

**Note**: This project was developed as part of the Qualcomm Arduino AI Challenge.
