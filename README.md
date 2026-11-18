Repository containing the code for an embedded system course.
This repo contains the code for a raspberry-py 5 that process the output of an AI camera IMX-500 and control two servo to track an animal with the camera

## Build

Requires CMake, pkg-config and libcamera (on Raspberry Pi OS: `sudo apt install cmake pkg-config libcamera-dev`).

```bash
cmake -S . -B build && cmake --build build -j4 && ./build/rilevamento_imx500
```
