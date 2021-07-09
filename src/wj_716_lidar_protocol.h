#ifndef WJ_716_LIDAR_PROTOCOL_H
#define WJ_716_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <wj_716_lidar/wj_716_lidarConfig.h>
#include <tf/transform_broadcaster.h>
using namespace std ;
namespace wj_lidar
{
  #define MAX_LENGTH_DATA_PROCESS 20000
  typedef struct TagDataCache
  {
    char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
  }DataCache;
  class wj_716_lidar_protocol
  {
  public:
    wj_716_lidar_protocol();
    bool dataProcess(const char *data,const int reclen);
    bool protocl(const char *data,const int len);
    bool OnRecvProcess(char *data, int len);
    bool checkXor(char *recvbuf, int recvlen);
    void send_scan(const char *data,const int len);
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    sensor_msgs::LaserScan scan;
    bool setConfig(wj_716_lidar::wj_716_lidarConfig &new_config,uint32_t level);
    //dynamic_reconfigure::Server<wj_safety_lidar_protocl::> dr_srv;
  private:
    char        data_[MAX_LENGTH_DATA_PROCESS];
    DataCache   m_sdata;
    wj_716_lidar::wj_716_lidarConfig config_;
    int m_n32PreFrameNo;
    float scandata[810];
    float scaninden[810];
		tf::TransformBroadcaster tf_broadcaster_;
  };
}
#endif // WJ_716_LIDAR_PROTOCOL_H
