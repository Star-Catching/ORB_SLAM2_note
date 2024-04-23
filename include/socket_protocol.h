//
// Created by hwh on 2022/9/16.
//

#ifndef PROTOCOL_SOCKET_PROTOCOL_H
#define PROTOCOL_SOCKET_PROTOCOL_H

#include <vector>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

#define SOCKET_PATH "/home/hwh/Data/ubuntu_code/path_socket"
#define BLOCK_MODE      1
#define NO_BLOCK_MODE   0
class socket_protocol
{
public:
    socket_protocol();
    void socket_init();
    void socket_recv();
    void socket_unpack();
    void socket_msg();
    void socket_send(string path);
    bool detect_point(uint16_t x, uint16_t y, float scale);
    void plot_roi(cv::Mat &im);

public:
    // 非阻塞模式 https://blog.csdn.net/mayue_web/article/details/82873115
    int sockfd;                     //socket句柄
    int position_cnt;               //接收到动态物体(框)的个数
    bool socket_mode = BLOCK_MODE;  //阻塞模式和非阻塞模式(该工程只能用阻塞模式)
    bool recv_sucess;               //是否接受成功
    bool socket_enable;             //是否使能动态点检测
    vector<char> send_buf;          //发送缓冲区 发送字符串
    vector<uint8_t> recv_buf;       //接受缓冲区 接受ASCII码
    vector<vector<uint16_t>> position;  //检测位置ROI
};

extern socket_protocol my_socket;
#endif //PROTOCOL_SOCKET_PROTOCOL_H
