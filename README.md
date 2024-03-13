# Image/Audio sensing and decision making for Kaia.ai robots

Implements this functionality:
- WebRTC setup
  - video, audio and data channels
  - a WebRTC-to-ROS2 bridge (TODO)
- Web server setup
  - a website for robot's smartphone/tablet (TODO)
  - SSL provider
- Robot face controller (TODO)
- Image processing (TODO)
- Audio processing (TODO)
- Decision making (TODO)

## PC Setup

- Windows PC setup instructions [here](https://kaia.ai/blog/local-pc-setup-windows/)
- Windows PC setup instructions [video](https://youtu.be/XOc5kCE3MC0).
- Linux PC setup
  - install Docker

- run `docker pull kaiaai/kaiaai:humble` or `docker pull kaiaai/kaiaai:iron`

## Instructions

```
# On your host PC, launch Kaia.ai Docker image
docker run --name makerspet -it --rm -p 8888:8888/udp -p 4430:4430/tcp -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 kaiaai/kaiaai:humble

# Launch SSL proxy
cd /py/ssl && ./ssl-proxy-linux-amd64 -from 0.0.0.0:4430 -to 127.0.0.1:8080 -redirectHTTP

# On your host PC, open one more bash session
docker exec -it makerspet bash

# Launch web server
cd /py/server && python3 server.py

# Open a browser on your host PC and navigate to https://localhost:4430
# Alternatively, open a browser on your smartphone/tablet and navigate to https://YOUR_HOST_PC_IP:4430
# Click YES/ALLOW/OK on your browser prompt to start video/audio/data WebRTC streaming
# A robot's animated interactive face should appear (TODO)
```
