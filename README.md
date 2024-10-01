# Kaia.ai Web stack

Web frontend and backend including aiohttp web server,
(TODO) React JS app(s), SSL, WebRTC image and audio,
WebRTC-to-ROS2 bridge for Kaia.ai robots.

Frontend implements this functionality:
- TODO React JS app(s)

Backend implements this functionality:
- TODO routes for web apps
- WebRTC setup
  - video, audio and data channels
  - a WebRTC-to-ROS2 bridge
- Web server setup
  - a website for robot's smartphone/tablet (TODO)
  - SSL provider
- Robot face controller (TODO)
- Image processing (TODO)
- Audio processing (TODO)
- Decision making (TODO)

## Setup Instructions

kaiaai_web is installed as part of the Kaia.ai build
using https://github.com/kaiaai/install

See https://github.com/kaiaai/kaiaai for general setup.

## Bringup Instructions

```
# On your host PC, launch Kaia.ai Docker image
docker run --name makerspet -it --rm -p 8888:8888/udp -p 4430:4430/tcp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaiaai:humble

# Launch SSL proxy
ros2 run kaiaai_python ssl_proxy.sh

# On your host PC, open one more bash session
docker exec -it makerspet bash

# Launch web server
ros2 run kaiaai_python web_server

# Open a browser on your host PC
#   Navigate to https://localhost:4430
# Open a browser on your smartphone/tablet
#   Navigate to https://YOUR_HOST_PC_IP:4430
# Click YES/ALLOW/OK on your browser prompt to start video/audio/data WebRTC streaming
#   A robot's animated interactive face should appear (TODO)
```

## Release notes

### v0.2.0
- renamed package from `kaiaai_python` to `kaiaai_web`

### v0.1.0
- set up aiohttp web server with aiortc WebRTC
- added WebRTC-to-ROS2 bridge
  - browser webcam video streams over WebRTC, gets published to ROS2
- ROS2 style web server parameters
  - logging from within web server to ROS2 does not work
- enabled SSL support using temporary certs
  - SSL certs replace launching a separate ssl-proxy
- web server launch file
- added [kaia-face.js](https://github.com/kaiaai/kaia-face.js) example
- added web cam FPS selection
