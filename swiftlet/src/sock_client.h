#ifndef CLIENT_H
#define CLIENT_H

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

#include "AP_controller.h"
#include "lidar.h"
#include "AP_interface.h"

#define buffer_size_header 60
#define buffer_size_data   30
#define buffer_size_action_header 30
#define buffer_size_PID_msg 25

using namespace std;

// TCP Client class

class tcp_client
{
private:
    int sock;
    string address;
    int port;
    struct sockaddr_in server;

public:
    string GS_CMD;

    tcp_client();
    bool conn(string, int);
    bool send_data(string data);
    string receive(int);
    int start_read_t = 0;
};

/*
int nChar(char* c);
void prepare_sock_msg_header(Hokuyo_lidar* lidar, Pixhawk_Interface* quad, double time_stamp, int mode, int RF, char* msg_header);
void prepare_sock_msg_data(Hokuyo_lidar* lidar, int idx, int data_end, char* msg_data);
void prepare_sock_action_header(string action, string param, char* msg_header);
void prepare_PID_msg(float kp, float ki, float kd, char* msg_header);
*/

#endif // CLIENT_H
