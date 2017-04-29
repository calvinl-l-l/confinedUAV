#include "sock_client.h"


tcp_client::tcp_client()
{
    sock = -1;
    port = 0;
    address = "";
}

/**
    Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }

        cout<<"Socket created\n";
    }
    else    {   /* OK , nothing */  }

    //setup address structure
    if(inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;

        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname\n";

            return false;
        }

        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;

        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];

            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;

            break;
        }
    }

    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }

    server.sin_family = AF_INET;
    server.sin_port = htons( port );

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }

    cout<<"Connected\n";
    return true;
}

/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        //perror("Send failed : ");
        cout << "failed to send" << endl;
        while (1){}
        return false;
    }
    //cout<<"Data send\n";

    return true;
}

/**
    Receive data from the connected host
*/
string tcp_client::receive(int size=512)
{
    char buffer[size];
    string reply;

    //Receive a reply from the server
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        puts("recv failed");
    }

    reply = buffer;
    return reply;
}



/*
//=========================== socket com helper functions ====================================================
void prepare_PID_msg(float kp, float ki, float kd, char* msg_header)
{

    char header[buffer_size_PID_msg];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;


    snprintf(header, buffer_size_PID_msg, "H %.3f %.3f %.3f ", kp, ki, kd);
    n = nChar(header);

    // calculating spacer value
    space = buffer_size_PID_msg - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_PID_msg, "%lld h", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_PID_msg;i++)
    {
        msg_header[i] = header[i];
    }

    //cout << "header " << header << endl;

}


void prepare_sock_action_header(string action, string param, char* msg_header)
{

    char header[buffer_size_action_header];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;

    //snprintf(header, buffer_size_action_header, "$ %c %c ", &action[0], &param[0]);

    strcat(header, "$ ");
    strcat(header, action.c_str());
    strcat(header, " ");
    strcat(header, param.c_str());
    strcat(header, " ");

    n = nChar(header);

    // calculating spacer value
    space = buffer_size_action_header - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_action_header, "%lld !", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_action_header;i++)
    {
        msg_header[i] = header[i];
    }
    //cout << "Action header: " << header << " n " << n << " space " << space << " total " << nChar(header) << endl;

}


// lidar h
void prepare_sock_msg_header(Hokuyo_lidar* ldata, Pixhawk_Interface* pixdata, double time_stamp, int mode, int RF, char* msg_header)
{
    RF = 0;

    char header[buffer_size_header];
    int n = 0;
    unsigned long long spacer = 0;
    int space = 0;

    if (ldata->y==NAN) ldata->y = 0;
    if (ldata->z==NAN) ldata->z = 0;

    snprintf(header, buffer_size_header, "H %d %.2f %.2f %.2f %.2f %.1f %d %.2f %.2f ", ldata->nyz, ldata->y, ldata->z, pixdata->roll, pixdata->yaw, time_stamp, mode, ldata->area, (float) RF/1000.0);
    n = nChar(header);

    // calculating spacer value
    space = buffer_size_header - n - 2;


    if (space >= 2)    spacer = 10;


    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;

    }


    // building header
    // s nraw y z roll yaw time mode
    snprintf(header + n, buffer_size_header, "%lld h", spacer);

    // transfering data to buffer
    for (int i=0;i<buffer_size_header;i++)
    {
        msg_header[i] = header[i];
    }

    //cout << "header " << header << endl;

}

// lidar h
void prepare_sock_msg_data(Hokuyo_lidar* ldata, int idx, int data_end, char* msg_data)
{
    char data[buffer_size_data];
    int n = 0;
    int space = 0;
    unsigned long long spacer = 0;

    snprintf(data, buffer_size_data, "D %.5f %.5f %d ", ldata->angle[idx], ldata->range[idx], data_end);
    n = nChar(data);


    // calculating spacer value
    space = buffer_size_data - n - 2;

    if (space >= 2)    spacer = 10;

    for (int i=0;i<space-2;i++)
    {
        spacer *= 10;
    }


    // building header
    snprintf(data + n, buffer_size_data, "%lld d", spacer);


    // transfering data to buffer
    for (int i=0;i<buffer_size_data;i++)
    {
        msg_data[i] = data[i];
    }
    //cout << "data -> " << data << endl;
}

// lidar h
int nChar(char* c)
{
    return string(c).length();
}
*/
