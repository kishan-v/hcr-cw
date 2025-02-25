Setup:
```bash
sudo apt-get install -y nvidia-container
docker pull nvcr.io/nvidia/l4t-base:r35.3.1
docker build -t bernie_transmitter:0.1.0 .
```

TODO: finalise docker run command. Something like:
```bash
docker run --rm --it --network=host --device=/dev/bus/usb/<bus>/<device> bernie_transmitter:0.1.0
```

Could also test camera with
```bash
mkdir -p /opt/hcr-cw/Videos/theta
gst-launch-1.0 thetauvcsrc mode=4K num-buffers=200 ! queue ! h264parse ! mp4mux ! filesink location=/opt/hcr-cw/Videos/theta/short_video.mp4
```