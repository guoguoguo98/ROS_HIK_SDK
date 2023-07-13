#include "HikCamera.h"

HikvisionCamera *HikvisionCamera::instance_ = nullptr;

HikvisionCamera::HikvisionCamera()
    : private_nh_("hik_camera"),
      image_transport_(private_nh_),
      connect_time_(2000),
      reconnect_time_(10000),
      user_id_(-1),
      play_handler_(-1)
{
    instance_ = this;
    initParams();
    initROSIO();
}

HikvisionCamera::~HikvisionCamera()
{
    releaseResources();
}

void HikvisionCamera::start()
{
    // 初始化HCNetSDK
    if (!initializeHCNetSDK())
    {
        ROS_ERROR("Failed to initialize HikCamera SDK.");
        return;
    }

    // 登录相机
    if (!loginCamera())
    {
        ROS_ERROR("Failed to login to the camera.");
        return;
    }

    ROS_INFO("Successfully logged in to the camera.");

    // 开始预览
    startPreview();

    // 接收数据循环
    receiveLoop();
}

void HikvisionCamera::initParams()
{
    // 读取参数
    private_nh_.param<std::string>("device_address", device_address_, "192.168.1.71");
    private_nh_.param<std::string>("user_name", user_name_, "admin");
    private_nh_.param<std::string>("password", password_, "123456");
    private_nh_.param<int>("asyn_login", asyn_login_, 0);
    private_nh_.param<int>("port", port_, 62222);
    private_nh_.param<int>("frame_width", frame_width_, 1920);
    private_nh_.param<int>("frame_height", frame_height_, 1080);
    private_nh_.param<std::string>("img_topic_name", img_topic_, "hik_img");

    // 相机信息
    private_nh_.param<std::string>("camera_name", camera_name, "hik_1");
    private_nh_.param<std::string>("camera_frame_id", camera_frame_id, "hik_1");
    private_nh_.param<std::string>("camera_info_url", camera_info_url, "");
}

void HikvisionCamera::initROSIO()
{
    // 创建图像发布者
    image_publisher_ = image_transport_.advertise(img_topic_, 1);

    // 创建相机信息管理器
    camera_info_ = new camera_info_manager::CameraInfoManager(private_nh_, camera_name, camera_info_url);

    // 如果未进行校准，则设置默认相机信息
    if (!camera_info_->isCalibrated())
    {
        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_->setCameraName(camera_name);
        camera_info_msg.header.frame_id = camera_frame_id;
        camera_info_msg.width = frame_width_;
        camera_info_msg.height = frame_height_;
        camera_info_->setCameraInfo(camera_info_msg);
    }
    // camera_info_service_ = private_nh_.advertiseService("set_camera_info", setCameraInfo);
}

bool HikvisionCamera::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp)
{
    sensor_msgs::CameraInfo &camera_info_msg = req.camera_info;
    camera_info_msg.header.frame_id = camera_frame_id;
    camera_info_->setCameraInfo(camera_info_msg);
    rsp.success = true;
    rsp.status_message = "Successfully set camera_info.";
    return true;
}

void HikvisionCamera::publishImage(const cv::Mat &image)
{
    // 将图像转换为ROS消息并发布
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_publisher_.publish(msg);
}

void HikvisionCamera::loadCameraParameters()
{
    // 设置连接和重新连接时间
    NET_DVR_SetConnectTime(connect_time_, 1);
    NET_DVR_SetReconnect(reconnect_time_, true);
}

bool HikvisionCamera::initializeHCNetSDK()
{
    // 初始化HCNetSDK
    if (NET_DVR_Init())
        return true;
    else
        return false;
}

bool HikvisionCamera::loginCamera()
{
    // 登录相机
    NET_DVR_USER_LOGIN_INFO login_info = {0};
    NET_DVR_DEVICEINFO_V40 device_info = {0};
    login_info.bUseAsynLogin = asyn_login_;
    login_info.wPort = port_;
    stpncpy(login_info.sDeviceAddress, device_address_.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
    stpncpy(login_info.sUserName, user_name_.c_str(), NAME_LEN);
    stpncpy(login_info.sPassword, password_.c_str(), NAME_LEN);

    user_id_ = NET_DVR_Login_V40(&login_info, &device_info);
    if (user_id_ < 0)
        return false;
    else
        return true;
}

void HikvisionCamera::startPreview()
{
    // 开始预览
    NET_DVR_PREVIEWINFO preview_info = {0};
    preview_info.hPlayWnd = 0;
    preview_info.lChannel = 1;
    preview_info.dwStreamType = 0;
    preview_info.dwLinkMode = 0;
    preview_info.bBlocked = 1;
    play_handler_ = NET_DVR_RealPlay_V40(user_id_, &preview_info, dataCallbackStatic, nullptr);
    if (play_handler_ < 0)
    {
        ROS_ERROR("Failed to start preview.");
        return;
    }
    else
    {
        ROS_INFO("Successfully started preview.");
    }
}

void HikvisionCamera::receiveLoop()
{
    // 接收数据循环
}

void HikvisionCamera::releaseResources()
{
    // 释放资源
    if (play_handler_ >= 0)
    {
        NET_DVR_StopRealPlay(play_handler_);
        play_handler_ = -1;
    }

    if (user_id_ >= 0)
    {
        NET_DVR_Logout(user_id_);
        user_id_ = -1;
    }

    NET_DVR_Cleanup();
}

void CALLBACK HikvisionCamera::dataCallback(LONG handle, DWORD dataType, BYTE *pBuffer, DWORD bufSize)
{
    int result = 0;
    switch (dataType)
    {
    case NET_DVR_SYSHEAD:
    {
        // 获取端口
        if (!PlayM4_GetPort(&port_))
        {
            ROS_ERROR("Failed to get port: %d", PlayM4_GetLastError(port_));
            break;
        }

        if (bufSize > 0)
        {
            // 设置实时流播放模式
            if (!PlayM4_SetStreamOpenMode(port_, STREAME_REALTIME))
            {
                break;
            }

            // 打开流并播放
            if (!PlayM4_OpenStream(port_, pBuffer, bufSize, frame_width_ * frame_height_))
            {
                result = PlayM4_GetLastError(port_);
                break;
            }

            // 设置解码回调
            if (!PlayM4_SetDecCallBack(port_, decodeCallbackStatic))
            {
                result = PlayM4_GetLastError(port_);
                break;
            }

            // 开始播放
            if (!PlayM4_Play(port_, 0))
            {
                result = PlayM4_GetLastError(port_);
                break;
            }
        }
        break;
    }

    case NET_DVR_STREAMDATA:
    {
        if (bufSize > 0 && port_ != -1)
        {
            // 输入数据
            if (!PlayM4_InputData(port_, pBuffer, bufSize))
            {
                break;
            }
        }
        break;
    }

    default:
    {
        if (bufSize > 0 && port_ != -1)
        {
            // 输入数据
            if (!PlayM4_InputData(port_, pBuffer, bufSize))
            {
                break;
            }
        }
        break;
    }
    }

    if (result != 0)
    {
        ROS_ERROR("Error Occurs when data callback: %d", result);
    }
}

void CALLBACK HikvisionCamera::decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo)
{
    cv::Mat bgr_image;
    if (pFrameInfo->nType == T_YV12)
    {
        // 转换YUV图像为BGR图像
        cv::Mat yuv_image(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (unsigned char *)pBuf);
        cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_YV12);
        cv::resize(bgr_image, bgr_image, cv::Size(frame_width_, frame_height_));
        // 发布图像
        publishImage(bgr_image);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hcnet_sdk");
    // 创建HikvisionCamera对象
    HikvisionCamera camera;
    // 调用初始化方法
    camera.start();
    ros::spin();
    return 0;
}
