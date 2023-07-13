#pragma once

#include <ros/ros.h>
#include <string.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
// opencv
#include <opencv2/opencv.hpp>
// 海康相机
#include "HCNetSDK.h"
#include "plaympeg4.h"

class HikvisionCamera
{
public:
    HikvisionCamera();  // 构造函数
    ~HikvisionCamera(); // 析构函数

    void start(); // 启动相机

    // 通过静态回调函数调用非静态成员函数
    static void CALLBACK decodeCallbackStatic(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo, void *nReserved1, int nReserved2)
    {
        if (instance_)
        {
            instance_->decodeCallback(nPort, pBuf, nSize, pFrameInfo);
        }
    }

    static void CALLBACK dataCallbackStatic(LONG handle, DWORD dataType, BYTE *pBuffer, DWORD bufSize, void *pUser)
    {
        if (instance_)
        {
            instance_->dataCallback(handle, dataType, pBuffer, bufSize);
        }
    }

private:
    void initParams();                                                                                       // 初始化参数
    void initROSIO();                                                                                        // 初始化ROS IO
    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp); // 设置相机信息
    void loadCameraParameters();                                                                             // 加载相机参数
    bool initializeHCNetSDK();                                                                               // 初始化海康SDK
    bool loginCamera();                                                                                      // 登录相机
    void startPreview();                                                                                     // 开始预览
    void receiveLoop();                                                                                      // 接收循环
    void releaseResources();                                                                                 // 释放资源
    void publishImage(const cv::Mat &image);                                                                 // 发布图像
    void CALLBACK decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo);                  // 解码回调函数
    void CALLBACK dataCallback(LONG handle, DWORD dataType, BYTE *pBuffer, DWORD bufSize);                   // 数据回调函数

private:
    ros::NodeHandle private_nh_;                          // ROS节点句柄
    image_transport::ImageTransport image_transport_;     // 图像传输
    image_transport::Publisher image_publisher_;          // 图像发布者
    camera_info_manager::CameraInfoManager *camera_info_; // 相机信息管理器
    ros::ServiceServer camera_info_service_;              // 相机信息服务

    std::string device_address_; // 设备地址
    std::string user_name_;      // 用户名
    std::string password_;       // 密码
    int asyn_login_;             // 异步登录
    int port_;                   // 端口号
    int frame_width_;            // 帧宽度
    int frame_height_;           // 帧高度
    std::string img_topic_;      // 图像话题

    // camera info
    std::string camera_name;     // 相机名称
    std::string camera_frame_id; // 相机坐标系
    std::string camera_info_url; // 相机信息URL

    int connect_time_;   // 连接时间
    int reconnect_time_; // 重新连接时间

    LONG user_id_;      // 用户ID
    LONG play_handler_; // 播放句柄

    static HikvisionCamera *instance_; // 静态实例指针
};
