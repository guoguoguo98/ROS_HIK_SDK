# HIK Camera ROS驱动

这是一个用于海康相机的ROS驱动程序。该驱动程序使用海康SDK与相机通信，并提供ROS节点和功能，以便在ROS系统中使用海康相机。

目前只实现了相机图像数据的发布`/hik_camera/hik_img` 和相机内参信息标定 `camera_info`。

## 安装依赖项

在使用本驱动程序之前，请确保你的系统满足以下依赖项：

- ROS（推荐使用 Melodic、Noetic 或更高版本）
- 海康SDK

## 获取和构建代码

使用以下步骤获取和构建代码：

1. 在ROS工作空间的src目录中克隆此代码库：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://172.24.0.51:8418/robot/hik_ros_driver.git
```

2. 进入工作空间的根目录，并编译代码：

```bash
cd ~/catkin_ws
catkin_make
```

## 配置相机

在使用相机之前，你需要进行一些配置。请按照以下步骤操作：

1. 编辑`hik_camera/config/camera.yaml`文件，设置相机的相关参数，如IP地址、端口号等。

2. 编辑`hik_camera/config/camera_info.yaml`文件，设置相机的标定信息和相机内参等。

   **可选操作：相机标定**

   如果你想进行相机标定以获取更精确的内参，可以执行以下步骤：

   - 使用ROS中的相机标定工具包（例如`camera_calibration`包）来捕获一系列不同角度和位置的图像，并记录相应的标定板点位数据。
   - 根据采集到的图像和标定板点位数据，运行相机标定算法以计算相机的内参矩阵和畸变系数。
   - 将计算得到的内参矩阵和畸变系数更新到`hik_camera/config/camera_info.yaml`文件中。

   ```bash
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/hik_camera/hik_img camera:=/hik_camera
   ```


3. 保存配置文件并关闭。


这样，其他用户在使用你的驱动程序时，可以根据需要选择是否执行相机标定步骤，以获得更精确的相机内参。
## 运行驱动程序

使用以下命令启动驱动程序节点：

```bash
roslaunch hik_camera camera.launch
```

请确保你已经进行了正确的ROS环境设置，并且相机已经连接到计算机。

## ROS节点

### `hik_camera_node`

该节点是驱动程序的核心节点，负责与相机进行通信并发布图像数据和相机信息。

#### 订阅的话题

- `/hik_camera/hik_img` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
  相机捕获的原始图像数据。

#### 发布的话题

- `/hik_camera/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
  相机的标定信息和内参。

#### 参数

- `~camera_info_url` (string, 默认值: "package://hik_camera/config/camera_info.yaml")
  相机标定信息的文件路径。

## 贡献

欢迎贡献代码和提出问题。如果你发现了bug，或者有任何改进的建议，请提交一个issue或者发送一个pull请求。

## 许可证

本驱动程序遵循 [MIT License](LICENSE)。

## 链接

- ROS官方网站: [http://www.ros.org/](http://www.ros.org/)
- 海康SDK文档: [https://www.hikvision.com/](https://www.hikvision.com/)