//
// Created by hwh on 2022/9/16.
//
#include <cmath>
#include <string>
#include <chrono>
#include <sys/un.h>
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include "socket_protocol.h"

using namespace std;

socket_protocol::socket_protocol()
{
    socket_enable = false;
    position_cnt  = -1;
    socket_mode   = BLOCK_MODE;
    recv_sucess   = false;
}
void socket_protocol::socket_init()
{
    if(!socket_enable){
        return;
    }
    //创建socket，指定通信协议为AF_UNIX,数据方式SOCK_STREAM
    if(socket_mode == BLOCK_MODE){
        //block mode
        sockfd = socket(AF_UNIX, SOCK_STREAM , 0);
    }else{
        //no block mode
        sockfd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    }
    if(sockfd==-1){
        cerr << "socket create fail" << endl;
        exit(EXIT_FAILURE);
    }
    //配置server_address
    struct sockaddr_un address;
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, SOCKET_PATH);
    //连接socket
    int result = connect(sockfd, (struct sockaddr *)&address, sizeof(address));
    if(result == -1){
        cout << "ensure the server is up" << endl;
        cerr << "socket connect fail" << endl;
        exit(EXIT_FAILURE);
    }
}

void socket_protocol::socket_send(string path)
{
    if(!socket_enable){
        return;
    }
    send_buf.clear();
    send_buf.push_back('w');       //帧头1
    send_buf.push_back('h');       //帧头2
    send_buf.push_back('m');       //ID: m:master 主机 s:slaver
    send_buf.push_back('0');       //function
    send_buf.push_back(' ');       //length
    //add path to socket_protocol::send_buf
    send_buf.insert(send_buf.end(),path.begin(),path.end());
    send_buf[4] = send_buf.size() - 5;
    if(write(sockfd, &send_buf[0], send_buf.size())==-1)
    {
        cout << "write error" << endl;
        exit(EXIT_FAILURE);
    }
    cout << "send success -- " << path << endl;
}
void socket_protocol::socket_unpack()
{
    if(recv_buf[0] == 0xaa && recv_buf[1] == 0xff){
        cout << "read success" << endl;
        recv_sucess = 1;
        position_cnt = recv_buf[2];
        position.clear();
//        position.resize(position_cnt,vector<uint16_t>(4,0));
        position.resize(position_cnt);
        uint16_t _cnt = 3;
        for(int i = 0; i < position_cnt; i++){      //遍历所有位置块
            for(int j = 0; j < 4; j++){             //每个位置块有4个点
                position[i].push_back((uint16_t)recv_buf[_cnt++] << 8 | recv_buf[_cnt++]);
            }
        }
    }else{
        recv_sucess = 0;
        cout << "read fail" << endl;
    }
}
void socket_protocol::socket_msg()
{
    if(recv_sucess == 1){
        for(int i = 0; i < position_cnt; i++){      //遍历所有位置块
            for(int j = 0; j < 4; j++){             //每个位置块有4个点
                cout << position[i][j] << " ";
            }
            cout << endl;
        }
    }else{
        cout << "no data" << endl;
    }
}
void socket_protocol::socket_recv()
{
    if(!socket_enable){
        return;
    }
    auto startTime = std::chrono::high_resolution_clock::now();
    cout << "ready to read" << endl;
    recv_sucess = 0;
    recv_buf.clear();
    recv_buf.resize(256);
    int flag = read(sockfd,&recv_buf[0],512);
    if(flag < 0){
        if(socket_mode == BLOCK_MODE){
            if(!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN)){
                cout << "read error" << endl;
                exit(EXIT_FAILURE);
            }
            cout << "block mode-----------" << endl;
        }else{
            cout << "read error" << endl;
            exit(EXIT_FAILURE);
        }
    }else{
        socket_unpack();
        socket_msg();
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = endTime - startTime;
    std::cout << "protocol time is " <<fp_ms.count() << "ms" << std::endl;
}

//检测point(x,y)是否在检测框内
bool socket_protocol::detect_point(uint16_t x, uint16_t y, float scale)
{
    //如果没有检测到动态物体则直接返回false
    if(position_cnt <= 0 || !socket_enable){
        return false;
    }
    //行检测
    bool row_check = false;
    for(int i = 0; i < position_cnt; i++){
        if(x >= (static_cast<float>(position[i][0]) * scale) && x <= (static_cast<float>(position[i][2]) * scale)){
            row_check = true;
            break;
        }
    }
    if(row_check == false){
        return false;
    }
    //列检测
    bool col_check = false;
    for(int i = 0; i < position_cnt; i++){
        if(y >= (static_cast<float>(position[i][1]) * scale) && y <= (static_cast<float>(position[i][3]) * scale)){
            col_check = true;
            break;
        }
    }
    return col_check == false ? false : true;
}
void socket_protocol::plot_roi(cv::Mat &im)
{
    //如果没有检测到动态物体则直接返回false
    if(position_cnt <= 0 || !socket_enable){
        return ;
    }
    for(uint8_t i = 0; i < position_cnt; i++){
        cv::rectangle(im,cv::Point(position[i][0],position[0][1]),cv::Point(position[i][2],position[0][3]),cv::Scalar(0, 255, 255));
    }
}