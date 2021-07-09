#include <ros/ros.h>
#include "async_client.h"
#include "wj_716_lidar_protocol.h"
using namespace wj_lidar;

int InitTcpConnection(const char* addr,int port,Async_Client **client_,fundata_t fundata_)
{

  io_service iosev;

  ip::tcp::endpoint ep(ip::address_v4::from_string(addr),port);
  *client_ = new Async_Client(iosev,ep,fundata_);

  iosev.run();

  return 1 ;
}

int boost_tcp_init_connection(const char* addr,int port,Async_Client **client_,fundata_t fundata_)
{
  int timecnt=0 ;
  *client_ = NULL ;
  boost::thread tmp(&InitTcpConnection,addr,port,client_,fundata_);
  tmp.detach() ;

  while(timecnt<50){
    timecnt++ ;
    usleep(20000); //20 ms
    if((*client_)->client_return_status()){
      return 0 ;
    }
  }
  *client_ = NULL ;
  return -1 ;

}

int boost_tcp_sync_send(Async_Client *client_ ,const char* msg,const int len)
{
  if(client_==NULL || client_->client_return_status()==0 ){
    printf("not connected , please connect first \n");
    return -1 ;
  }
  else{
    client_->client_async_write((char*)msg,len);
    return 0 ;
  }

  return 1;
}

int boost_tcp_sync_read(Async_Client *client_ )
{
  if(client_==NULL || client_->client_return_status()==0 ){
    printf("not connected , please connect first \n");
    return -1 ;
  }
  else{
    client_->client_async_read();
    return 0 ;
  }
  return 1;
}

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------ */
wj_716_lidar_protocol *protocol;
Async_Client *client;
void CallBackRead(const char* addr,int port,const char* data,const int len)
{
  protocol->dataProcess(data,len);
}

void callback(wj_716_lidar::wj_716_lidarConfig &config,uint32_t level)
{
  protocol->setConfig(config,level);
}

//void timerCallback(const ros::TimerEvent&)
//{
//  char scandata[5]={0x02,0x02,0x02,0x02,0x00};
//  //cout << "20ms" <<endl;
//  boost_tcp_sync_send(client,scandata,5);
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wj_716_lidar_01");
  ros::NodeHandle nh("~");
//  ros::Timer timer;
  std::string hostname;
  nh.getParam("hostname",hostname);
  cout << hostname <<endl;
  std::string port;
  nh.getParam("port",port);
  port = "2110";


  protocol = new wj_716_lidar_protocol();
  dynamic_reconfigure::Server<wj_716_lidar::wj_716_lidarConfig> server;
  dynamic_reconfigure::Server<wj_716_lidar::wj_716_lidarConfig>::CallbackType f;
  f = boost::bind(&callback,_1,_2);
  server.setCallback(f);
  client=NULL;

  boost_tcp_init_connection(hostname.c_str(),atoi(port.c_str()),&client,&CallBackRead);
//  timer= nh.createTimer(ros::Duration(0.02), timerCallback);   //ding shi fa song yao shu zhi ling
  boost_tcp_sync_read(client);

  ROS_INFO("Hello wj_716_lidar!");
  ros::spin();

}
