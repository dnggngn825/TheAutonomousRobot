DEVICE=/dev/video0
echo "Now adjusting the camera settings for $DEVICE"
v4l2-ctl --device=/dev/video0 --set-ctrl=focus_auto=0
v4l2-ctl --device=/dev/video0 --set-ctrl=focus_absolute=0
v4l2-ctl --device=/dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=1
echo " "
echo "Camera settings were adjusted to:"
v4l2-ctl --device=/dev/video0 --get-ctrl=focus_auto,focus_absolute
v4l2-ctl --device=/dev/video0 --get-fmt-video
