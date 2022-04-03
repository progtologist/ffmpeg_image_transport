# ffmpeg_image_transport

This ROS2 package allows one to use the ffmpeg encoders and decoders to compress the video stream from cameras to be transmitted over WIFI or cable connections.

## Installation

Simply clone the repository into a ROS2 workspace.

### Dependencies

Use your well known `rosdep` command to install all dependencies.
```
rosdep install -y --from-paths src --ignore-src
```
### Simple Building

Simply perform `colcon build` with your favorite arguments on that workspace. This will use your system-wide ffmpeg (libav) installation. By default, on Ubuntu flavors ffmpeg comes with VA-API and VDPAU hardware-acceleration enabled, so if your system is configured to use those, then you can use them with ROS2 as well.

### Advanced Building

On Ubuntu, ffmpeg is precompiled without support for NVENC/NVDEC. In nVidia's website there are instructions to build ffmpeg from scratch and install it system-wide with hardware acceleration enabled for their GPUs. 

As an alternative option, this package can automate this process and build a local (to the ROS2 workspace) version that has enabled NVENC/NVDEC.

To do this, you need to compile with
```
colcon build --symlink-install --cmake-args -DCOMPILE_FFMPEG=ON
```
This downloads version 4.2.4 version of ffmpeg (the same version that Focal uses) and attempts to build it with all hardware-acceleration modes enabled (if supported).

Unfortunately, to this point, I have not been able to make NVENC work properly and debugging this can be problematic (dlopen nvidia libraries precompiled without debug symbols). If someone with better experience wants to help to make this work better, feel free to submit an issue or make a pull request.

## Configuration

The encoders provide the following (reconfigurable) ROS parameters
- Bitrate (integer)
- Maximum Quantizer (integer)
- Camera FPS Parameter (string)
- Codec Name (string)

The `Codec Name`, `Bitrate` and `Maximum Quantizer` are parameters for the encoder. You can get a list of all the encoders of ffmpeg using `ffmpeg -encoders`. Keep in mind that video encoders are only the ones that start with a `V`. Commonly used encoders are `libx264` and `libx265`. 

The list will also printout the hardware accelerated versions of these encoders, however this does not mean that your hardware supports them, test and find the best configuration for your setup and hardware.

Typical rule of thumb for hardware acceleration for GPUs are:
- Intel -> VA-API, QSV
- AMD -> VA-API, AMF
- nVidia -> NVENC, VDPAU

The `Camera FPS Parameter` is the (local) path to the parameter that specifies the framerate of the camera driver. For example, if you are using the [usb_cam](https://github.com/ros-drivers/usb_cam/tree/ros2) driver then that value should be `framerate` because that is the key that `usb_cam` uses to specify the framerate of the camera.

The subscriber does not provide any configurable parameters, it automatically picks up the encoding algorithm from the ROS2 message and instantiates the appropriate decoder to decompress it.

## Contributions

Contributions are more than welcome, especially if you can provide better support for hardware-accelerated encoding/decoding as this will improve the usability of the transport further more.