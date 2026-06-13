# Surveillance System v2.0 - C++ High Performance

A professional-grade, bare-metal optimized surveillance system for Raspberry Pi using C++17, V4L2, and OpenCV.

## Features

- **V4L2 DMA Capture**: Zero-copy kernel buffers for minimal CPU overhead
- **Hardware H.264 Encoding**: GPU-accelerated video compression
- **Advanced Motion Detection**: MOG2 background subtraction with contour analysis
- **Dual Detection**: PIR sensor + Computer vision for reliability
- **Auto Night Vision**: IR LED control based on scene brightness
- **MJPEG Streaming**: Multi-client HTTP server (access from any browser)
- **Thread-safe Logging**: Lock-free ring buffer logger
- **Telegram/Email Alerts**: Instant notifications on motion
- **Auto-cleanup**: Age-based recording deletion
- **Systemd Service**: Auto-start on boot

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Raspberry Pi | 3B+/4/5 (4GB RAM recommended) |
| Camera | Pi Camera Module v2/v3 or USB camera |
| PIR Sensor | HC-SR501 or AM312 |
| IR LED Array | 940nm, 48-LED (optional) |
| Power Supply | 5V 3A USB-C |
| Storage | 32GB+ MicroSD or USB drive |

## GPIO Pinout

| Function | BCM Pin | Physical Pin |
|----------|---------|--------------|
| PIR VCC | - | Pin 2 (5V) |
| PIR GND | - | Pin 6 (GND) |
| PIR OUT | GPIO 17 | Pin 11 |
| IR LED | GPIO 18 | Pin 12 |
| Buzzer | GPIO 27 | Pin 13 |
| Status LED | GPIO 22 | Pin 15 |

## Quick Install

```bash
curl -sSL https://raw.githubusercontent.com/yourrepo/install.sh | bash
```

Or manually:

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev libgpiod-dev libv4l-dev
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo systemctl enable surveillance
sudo systemctl start surveillance
```

## Access Stream

Open any browser and navigate to:
```
http://raspberry-pi-ip:8080
```

## Architecture

```
┌─────────────────────────────────────────┐
│  CAPTURE THREAD (V4L2 DMA)              │
│  ├─ Zero-copy mmap buffers               │
│  └─ YUYV/MJPEG → OpenCV Mat              │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│  PROCESSING THREAD                      │
│  ├─ Night vision auto-adjust            │
│  ├─ MOG2 motion detection               │
│  ├─ Contour analysis & bounding boxes   │
│  └─ Frame pool management               │
└─────────────────┬───────────────────────┘
                  │
    ┌─────────────┼─────────────┐
    │             │             │
┌───▼───┐   ┌────▼────┐  ┌────▼────┐
│STREAM │   │ RECORD  │  │  ALERT  │
│THREAD │   │ THREAD  │  │ THREAD  │
│MJPEG  │   │ H.264   │  │ GPIO/   │
│HTTP   │   │ OpenCV  │  │ Telegram│
└───────┘   └─────────┘  └─────────┘
```

## File Structure

```
surveillance/
├── surveillance.hpp    # Main header with all classes
├── main.cpp            # Entry point and system orchestration
├── logger.cpp          # Thread-safe async logger
├── gpio.cpp            # libgpiod hardware control
├── camera.cpp          # V4L2 capture with DMA
├── motion.cpp          # Background subtraction detector
├── recorder.cpp        # Video recording with timestamp overlay
├── stream.cpp          # HTTP MJPEG server
├── alert.cpp           # Notification manager
├── CMakeLists.txt      # Build configuration
├── install.sh          # Automated installer
└── README.md           # This file
```

## License

MIT License - For educational and personal use only.
