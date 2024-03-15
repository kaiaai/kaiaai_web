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

## PC Setup Instructions

- Windows PC setup instructions [here](https://kaia.ai/blog/local-pc-setup-windows/)
- Windows PC setup instructions [video](https://youtu.be/XOc5kCE3MC0).
- Linux PC setup
  - install Docker

- run `docker pull kaiaai/kaiaai:humble` or `docker pull kaiaai/kaiaai:iron`

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
