# usb_cam_utils
My usb camera's codes. 
This repo includes:

## rgb_driver
Start a usb camera and publish the topic.

### Usage
```bash
roslaunch rgb_driver rgb_driver.launch
```

### Setting
- `video_port`: default 0. If you have many cameras (like a built-in cam), you may need change this value to 2 or 3.
- `fps`: camera's fps. Some USB cameras cannot change this value, so the `actual-fps` may be different.
- `exposure`: exposure time in s. 0 for auto-exposure, others for exposure time (e.g., 0.02 is 20ms). If you change to auto-exposure, you need to reset camera by reconnection.
- `output_rate`: ros publish image topic rate. If the rate is larger than `actual-fps`, the published images may be duplicated.
- `show_image`: show image in a new window. Press 'q' to quit showing.

### Features
- The usb camera's buff is always read by a new thread. Thus the published images are the newest.