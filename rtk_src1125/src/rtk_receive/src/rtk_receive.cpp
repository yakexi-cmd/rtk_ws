// #include "rtk_receive_node.hpp"

// using namespace std;
// // socket
// rtk_info::rtk_info()
// {

// }

// rtk_info::~rtk_info()
// {

// }

// void rtk_info::receive_msg()
// {
//     // 创建socket
//     // # socket意思是套接字,套接字是一种抽象层,可以通过套接字接口发送和接收数据
//     // # SOCK_DGRAM代表创建的是数据报(datagram对应dgram)套接字(socket)
//     // # AF_INET代表使用IPV4地址协议,AF(address family地址族),INET代表internet,AF_INET6则使用IPV6地址协议
//     // # udp通信没有主从定义，此处象征性称电路板为host服务器而本地为client客户端,实际双方IP都是固定的没有主次
//     int sockfd = socket(AF_INET,SOCK_DGRAM,0);//创建socket，确定ipv4的套结字，SOCK_DGRAM表示面向udp传输
//     struct sockaddr_in servaddr,cliaddr;//定义了一个结构体，结构体中存放ip
//     char buf[1024]={0};
    
//     if (sockfd<0)
//     {
//         cout<<"socket创建失败"<<endl;
//         return;
//     }

//     // 创建服务端的信息结构体
//     string ip = "192.168.3.129";
//     servaddr.sin_port = htons(7000);
//     // servaddr.sin_port = htons(INADDR_ANY);
//     servaddr.sin_family=AF_INET;
//     servaddr.sin_addr.s_addr=inet_addr(ip.c_str());
//     socklen_t len_servaddr= sizeof(servaddr);
//     socklen_t len_cliaddr = sizeof(cliaddr);//用于存储客户端地址的长度
//     // // 绑定socket,客户端由于存在一对多的情况，因此端口号是随机分配的，不需要bind
//     // if(bind(sockfd,(const struct sockaddr *)&servaddr,sizeof(servaddr))<0)
//     // {
//     //     cout<<"绑定失败"<<endl;
//     //     while(true)
//     //     {
//     //         cout<<"重新尝试绑定"<<endl;
//     //         sleep(1);
//     //         if(bind(sockfd,(const struct sockaddr *)&servaddr,sizeof(servaddr))>=0)
//     //         {
//     //             cout<<"重新绑定成功"<<endl;
//     //             break;
//     //         }
//     //     }
//     //     close(sockfd);
//     //     return;
//     // }
//     string message="send message";
//     while(ros::ok())
//     {
//         sendto(sockfd,message.c_str(),message.size(),0,(struct sockaddr *)&servaddr,len_servaddr);
        
//         /*
//             recvfrom用于接收udp数据，参数说明
//             sockfd：socker文件描述符
//             （char* ）buffer：接收数据的缓冲区
//             sizeof(buffer):缓冲区的大小
//             (struct sockaddr *)&cliaddr 指向cliaddr的指针，用于存储发送数据的客户端地址       
//         */
//         int n = recvfrom(sockfd,(char*)buf,sizeof(buf),0,(struct sockaddr *)&cliaddr,&len_cliaddr);
//         if(n<0)
//         {
//             cout<<"receive error!"<<endl;
//             break;
//         }
//         buf[n] = '\0';//保证字符串以null结尾
//         gps_read(gps_data,buf,sizeof(buf));
//         cout<<"在本次运行时得到的buf是:"<<buf<<endl;
//     }
//     close(sockfd);
// }

// void rtk_info::gps_read(&gps_data,(char[])& buf,sizeof(buf))
// {
//     gps_data=buf;
//     cout<<"gps_data:"<<gps_data<<endl;
// }

#include "rtk_receive_node.hpp"

using namespace std;
// socket
rtk_info::rtk_info()
{

}

rtk_info::~rtk_info()
{

}

void rtk_info::receive_msg()
{
    // 创建socket
    // # socket意思是套接字,套接字是一种抽象层,可以通过套接字接口发送和接收数据
    // # SOCK_DGRAM代表创建的是数据报(datagram对应dgram)套接字(socket)
    // # AF_INET代表使用IPV4地址协议,AF(address family地址族),INET代表internet,AF_INET6则使用IPV6地址协议
    // # udp通信没有主从定义，此处象征性称电路板为host服务器而本地为client客户端,实际双方IP都是固定的没有主次
    int sockfd = socket(AF_INET,SOCK_DGRAM,0);//创建socket，确定ipv4的套结字，SOCK_DGRAM表示面向udp传输
    struct sockaddr_in servaddr,cliaddr;//定义了一个结构体，结构体中存放ip
    char buf[1024]={0};
    
    if (sockfd<0)
    {
        cout<<"socket创建失败"<<endl;
        return;
    }

    // 创建服务端的信息结构体
    string ip = "192.1.1.24";
    servaddr.sin_port = htons(7000);
    // servaddr.sin_port = htons(INADDR_ANY);
    servaddr.sin_family=AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(ip.c_str());
    socklen_t len_servaddr= sizeof(servaddr);
    socklen_t len_cliaddr = sizeof(cliaddr);//用于存储客户端地址的长度
    // // 绑定socket,客户端由于存在一对多的情况，因此端口号是随机分配的，不需要bind
    // if(bind(sockfd,(const struct sockaddr *)&servaddr,sizeof(servaddr))<0)
    // {
    //     cout<<"绑定失败"<<endl;
    //     while(true)
    //     {
    //         cout<<"重新尝试绑定"<<endl;
    //         sleep(1);
    //         if(bind(sockfd,(const struct sockaddr *)&servaddr,sizeof(servaddr))>=0)
    //         {
    //             cout<<"重新绑定成功"<<endl;
    //             break;
    //         }
    //     }
    //     close(sockfd);
    //     return;
    // }
    string message="send message";
    while(ros::ok())
    {
        sendto(sockfd,message.c_str(),message.size(),0,(struct sockaddr *)&servaddr,len_servaddr);
        
        /*
            recvfrom用于接收udp数据，参数说明
            sockfd：socker文件描述符
            （char* ）buffer：接收数据的缓冲区
            sizeof(buffer):缓冲区的大小
            (struct sockaddr *)&cliaddr 指向cliaddr的指针，用于存储发送数据的客户端地址       
        */
        int n = recvfrom(sockfd,(char*)buf,sizeof(buf),0,(struct sockaddr *)&cliaddr,&len_cliaddr);
        if(n<0)
        {
            cout<<"receive error!"<<endl;
            break;
        }
        buf[n] = '\0';//保证字符串以null结尾
        // gps_read(gps_data,buf);
        cout<<"在本次运行时得到的buf是:"<<buf<<endl;
    }
    close(sockfd);
}
// void rtk_info::gps_read(string &gps_data,(string)& buf)这个写法是错误的，因为(string)& buf是函数类型转换，不能放在函数参数声明中
void rtk_info::gps_read(string &gps_data,char* buf)
{
    gps_data=string(buf);
    vector<string> tokens;
    string token;
    stringstream ss(gps_data);

    while(getline(ss,token,','))
    {
        if(token.empty()) token="0";
        cout<<"token:"<<token<<endl;
        tokens.push_back(token);
    }

    cout<<"tokens.size():"<<tokens.size()<<endl;
    if(tokens.size()>4)
    {
        try{
            float time_=stof(tokens[2]);
            float latitude_ = stof(tokens[3]);
            float longitude_ = stof(tokens[4]);
            float height_ = stof(tokens[5]);
            cout<<"时间t:"<<time_<<endl;
            cout<<"维度信息:"<<latitude_<<",经度信息:"<<longitude_<<",高度信息:"<<height_<<endl;
        }catch(const std::exception &e){
            std::cerr<<"解析错误"<<e.what()<<endl;
        }
    }
    else
    {
        cerr<<"数据格式错误或缺失字段"<<endl;
    }
    
}