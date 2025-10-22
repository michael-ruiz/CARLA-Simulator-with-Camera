# CARLA Simulator — Intel RealSense Camera Integration

Integrates an Intel RealSense camera with the CARLA driving simulator to enable in-vehicle monitoring and sensor testing. Includes additions compatible with CARLA / Unreal Engine 5.5.

Key points
- Integrates Intel RealSense SDK 2.0 with CARLA (tested with CARLA 0.9.16)
- Example scripts that extend CARLA/PythonAPI examples to work with a RealSense camera
- Works on Linux (development done on Ubuntu)

Requirements
- CARLA simulator (recommended 0.9.16; for Unreal Engine 5.5 use the UE5 build)
  - https://github.com/carla-simulator/carla
- Intel RealSense SDK 2.0 (librealsense + pyrealsense2)
  - https://dev.realsenseai.com/docs/get-started-in-sdk-20
- Python 3.8+ (system packages and pip)
- Typical Linux development tools (build deps for CARLA / Unreal as required)

Quick start (Linux)
1. Install Intel RealSense SDK and Python bindings (official instructions above). Example (may require repository add):
   sudo apt-get update
   sudo apt-get install -y librealsense2-dkms librealsense2-utils
   pip3 install pyrealsense2
2. Install or build the CARLA server appropriate for your CARLA build (0.9.16 / UE5). Start the server:
   - For UE5 build (if present):
     ./CarlaUE5.sh & 
   - For UE4 build:
     ./CarlaUE4.sh &
3. Install Python dependencies used by the examples (if a requirements file exists):
   pip3 install -r requirements.txt
   (or at minimum: pip3 install carla numpy opencv-python)
4. Run an example script from this repository:
   python3 examples/<example_script>.py
   Replace `<example_script>.py` with the example you want to run. Check script-level comments for device selection and config.

Repository layout (high level)
- examples/ — modified CARLA PythonAPI example scripts demonstrating RealSense integration
- README.md — this file

Notes and tips
- Confirm which CARLA executable your build provides (CarlaUE4.sh vs. CarlaUE5.sh). Use the matching executable.
- If you have multiple RealSense devices, set the device serial in example scripts or pass it via script arguments.
- For Unreal Engine 5.5 changes, check the examples directory for any UE5-specific adjustments.

References
- CARLA: https://github.com/carla-simulator/carla
- Intel RealSense SDK 2.0: https://dev.realsenseai.com/docs/get-started-in-sdk-20

Acknowledgements
- CARLA project
- Intel RealSense project