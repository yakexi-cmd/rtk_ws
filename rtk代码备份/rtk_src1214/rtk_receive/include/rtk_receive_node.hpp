#ifndef RTK_RECEIVE_NODE_H
#define RTK_RECEIVE_NODE_H

// ros相关库调用
#include <ros/ros.h>
#include <std_msgs/String.h>
// c++常用函数库
#include <iostream>
#include <cstdlib>
#include <string>
#include <Eigen/Dense> //Eigen::Vector3d是一个3D向量，来自Eigen库
// socket通信相关函数库
#include <sys/socket.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
using namespace std;


class rtk_info
{
public:
	rtk_info();
	~rtk_info();
	void receive_msg_udp();
	void gps_read(string &gps_data,char* buf);

private:
	// 读取FVI中的数据信息
	Eigen::Vector3d position;//存放经度，维度，海拔信息
	Eigen::Vector3d euler_angle;//存放roll(滚转),pitch(俯仰),yaw(偏航)
	Eigen::Vector3d velocity;//存放向东，向北，向天的速度信息
	string gps_data;
	string buf;
	// 读取RTKSTAT：定位天线收星信息
	
	// 读取ATTSTAT：测向副天线收星信                                                                                                                                       


	/*
	读取FVI综合语句中的信息，包括
		1.HHMMSS.SS:UTC时间
		2.DD.DDDDD:维度（度，正值为北方）
		3.DD.DDDDD:经度（度，正值为东方）
		4.AA.AAA:高程，海拔
		5.E.E：标准纬度偏差（m）
		6.F.F：标准经度偏差（m）
		7.F.F：标准高度偏差（m）
		8.HHH.HHH航向角（度，与正北方的夹角，由主天线指向副天线）
		9.hh.hhh:航向角标准偏差
		10.PP.PP:俯仰角
		11.pp.ppp:俯仰角标准偏差
		12.RR.RRR:横滚角（单位度）
		13.rr.rrr:横滚角标准偏差
		14.ve.eee：东向速度（m/s）
		15.vn.nnn:北向速度（m/s）
		16.vu.uuu:天向速（m/s）
		17.vv.vvv:对地速率
		18.LE.EEE:东向位置坐标
		19.LN.NNN:北向位置坐标
		20.LU.UUU:天向位置坐标
		21.ZONE:高斯投影区域
		22.UEEE.EEEE:高斯投影东向坐标(m)
		23.UNNN.NNNN:高斯投影北向坐标(m)
		24.PN:主天线卫星数
		25.p:定位状态指示
		26.h:定向状态指示	
		27.L:主副天线距离
		28.sss:差分延时
		29.**CC:校验和
		30.<CR><LF>:回车换行符
	*/


};
#endif