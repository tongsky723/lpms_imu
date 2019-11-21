#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_srvs/Empty.h>
#include <lpms_imu/serial_port.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>
//#include <XmlRpcValue.h>
//#include<XmlRpcException.h>
using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作
                     //定义字符串长度，IMU返回的数据是17个字节一组，可用串口调试助手获得


typedef float float32_t;
struct _sensorData
{

  float32_t gAngle;
  float32_t gRate;
  float32_t accX;
  float32_t accY;
  float32_t accZ;
} sensorData;

union cArray2intArray
{
  int16_t i[5];
  uint8_t c[10];
}c2i;

bool parse_data(std::vector<u_char>& dataBuffer)
{
  uint8_t function;
  uint8_t index;
  uint8_t length;
  int16_t angle;
  int16_t rate;
  int16_t x_acc;
  int16_t y_acc;
  int16_t z_acc;
  uint8_t check_sum = 0;
// Check header byte
  if (dataBuffer.at(0) != 0x3A)
  {
// Error
    ROS_ERROR("Head is not 0x3A");
    return false;
  }

  function = dataBuffer.at(1);
  index = dataBuffer.at(2);
  length = dataBuffer.at(3);
  memcpy(c2i.c, &dataBuffer.at(4), 10);
//Verify checksum
  for (int i = 1; i < 14; ++i)
    check_sum +=dataBuffer.at(i);
  if (check_sum != dataBuffer.at(14))
  {
    ROS_ERROR("check_sum is wrong");
    for(size_t i=0;i<17;++i)
    {
     printf("%02x",dataBuffer.at(i));
    }
    printf("::\r\n");
    return false;
  }
  /***********************************************************************************************

  包头  指令号  索引  数据长度    航向角    角速度    accX    accY    accZ   校验和  包尾低高字节
   3a  0b     11    0a=10     (3a ca)  (01 00) (5c 00) (fb ff) (0a fc)  87       0d 0a      一帧数据

  *************************************************************************************************/
/*
 * Standard Unit for lpms-nav2
 * Heading　deg   (need to divide by 100)
 * Angular Velocity  deg/s  (need to divide by 50)
 * Linear Acceleration   g  (need to divide by 1000)
 * */


//Scale and store data
  sensorData.gAngle = c2i.i[0] / 100.0;// angle / 100.0;
  sensorData.gRate = c2i.i[1] / 50.0;// rate / 50.0;
  sensorData.accX= c2i.i[2] / 1000.0;//x_acc;
  sensorData.accY = c2i.i[3] / 1000.0;//y_acc;
  sensorData.accZ = c2i.i[4] / 1000.0;//z_acc;
  return true;
}

//deg to rad
 double deg2rad(double deg)
{
    return deg * M_PI / 180.;
}


class LPMS_NAV2
{
//    define all variables in Class here, separate them into different group (private/public) based on their attribute
public:
    LPMS_NAV2();

    ~LPMS_NAV2();

    void run();
    SerialPort sp;

    int imu_rate=0;
    bool flag_port=false;
private:

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    std::string imu_port="";
    int imu_port_baud=0;
    bool publish_tf_=false;
    double init_heading=0.0;
    double current_heading=0.0;
    std::string imu_frame_id="";
    std::string fix_frame_id="";
    tf::TransformBroadcaster br;
    bool set_zero_orientation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    ros::Publisher imu_pub;
    ros::ServiceServer service_set;
    double g_factor;
};

LPMS_NAV2::LPMS_NAV2():
        private_nh("~")
{
// initialise all variables here
    private_nh.param<std::string>("imu_port",imu_port,"/dev/ttyUSB0");
    private_nh.param<int>("imu_rate",imu_rate,100.);
    private_nh.param<int>("imu_port_baud",imu_port_baud,115200.);
    private_nh.param<bool>("publish_tf",publish_tf_,false);
    private_nh.param<std::string>("imu_frame_id",imu_frame_id,"imu_link");
    private_nh.param<std::string>("fix_frame_id",fix_frame_id,"base_link");

    service_set = private_nh.advertiseService("set_zero", &LPMS_NAV2::set_zero_orientation, this);

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
    int try_cnt = 2;
    g_factor = 9.8;
    flag_port = sp.start(imu_port.c_str(),imu_port_baud);
    while (!flag_port)
    {
        this->sp.reset();
        usleep(2*10e4);
        if (try_cnt < 0)
        {
            ROS_FATAL("Could not open port, exiting");
            break;
        }
        ROS_WARN("Can not open port %s, retry count down --> %d", imu_port.c_str(), try_cnt);
        flag_port = sp.start(imu_port.c_str(),imu_port_baud);
        try_cnt = try_cnt - 1;
        sleep(1);
    }

    ROS_INFO("open :%s succeed,imu rate:%d,imu_frame_id:%s",imu_port.c_str(),imu_rate,imu_frame_id.c_str());
}


LPMS_NAV2::~LPMS_NAV2() {
    printf("node shutting down");
}

void LPMS_NAV2::run()
{
    std::vector<u_char> data;
    std::vector<u_char> buf;
    if(this->sp.get_data(&data))
    {
        if(data.at(0)==0x3a&&data.at(1)==0x0b)//如果头是3a，直接取17个数据
        {
            buf.assign(data.end()-17,data.end());//取第2帧数据
        }
        else
        {
            for(uint8_t i=1;i<34-1;++i)//第一个确定不是0x3a
            {
                if(data.at(i)==0x3a&&data.at(i+1)==0x0b)//:3a0b110a3aca01005c00fbff0afc870d0a一帧数据
                {
                    buf.assign(data.begin()+i,data.begin()+i+17);
                    break;//找到数据头，跳出循环
                }
            }
        }
        if(parse_data(buf))
        {
            geometry_msgs::TransformStamped trf;

            // To make sure get a consistent time in simulation
            ros::Time::waitForValid();
            current_heading = deg2rad(sensorData.gAngle);//rad/s

            double angular_velocity_z=deg2rad(sensorData.gRate);//角速度单位是rad/s 1.042
            double yaw;
            yaw = current_heading - this->init_heading;
            if (publish_tf_)
            {
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, yaw);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fix_frame_id, imu_frame_id));
            }

            if(fabs(angular_velocity_z)<=0.001) angular_velocity_z=0.f;

            double acc_x=sensorData.accX/9.8;//传感器单位是g,ros acc单位是m/s^2
            double acc_y=sensorData.accY/9.8;
            double acc_z=sensorData.accZ/9.8;
            //tf::Quaternion q= tf::createQuaternionFromRPY(0.0,0.0,yaw);//roll pitch yaw 转四元素
            tf::Quaternion q=tf::createQuaternionFromYaw(yaw);

            sensor_msgs::Imu imu;
//            imu.orientation = tf::createQuaternionMsgFromYaw(yaw);
            imu.orientation.x=q[0];
            imu.orientation.y=q[1];
            imu.orientation.z=q[2];
            imu.orientation.w=q[3];
            imu.header.stamp= ros::Time::now();
            imu.header.frame_id=imu_frame_id;
            // Accelerations should be in m/s^2 (not in g's)
            imu.linear_acceleration.x=acc_x * g_factor;
            imu.linear_acceleration.y=acc_y * g_factor;
            imu.linear_acceleration.z=-acc_z * g_factor;  // don't ask my why negative, ROS take z-linear acceleration round 9.8
            imu.angular_velocity.z=angular_velocity_z;

            imu.angular_velocity_covariance[0] = 0.02;
            imu.angular_velocity_covariance[1] = 0;
            imu.angular_velocity_covariance[2] = 0;
            imu.angular_velocity_covariance[3] = 0;
            imu.angular_velocity_covariance[4] = 0.02;
            imu.angular_velocity_covariance[5] = 0;
            imu.angular_velocity_covariance[6] = 0;
            imu.angular_velocity_covariance[7] = 0;
            imu.angular_velocity_covariance[8] = 0.02;

            imu.linear_acceleration_covariance[0] = 0.04;
            imu.linear_acceleration_covariance[1] = 0;
            imu.linear_acceleration_covariance[2] = 0;
            imu.linear_acceleration_covariance[3] = 0;
            imu.linear_acceleration_covariance[4] = 0.04;
            imu.linear_acceleration_covariance[5] = 0;
            imu.linear_acceleration_covariance[6] = 0;
            imu.linear_acceleration_covariance[7] = 0;
            imu.linear_acceleration_covariance[8] = 0.04;

            imu.orientation_covariance[0] = 0.025;
            imu.orientation_covariance[1] = 0;
            imu.orientation_covariance[2] = 0;
            imu.orientation_covariance[3] = 0;
            imu.orientation_covariance[4] = 0.025;
            imu.orientation_covariance[5] = 0;
            imu.orientation_covariance[6] = 0;
            imu.orientation_covariance[7] = 0;
            imu.orientation_covariance[8] = 0.025;

            this->imu_pub.publish(imu);
        }
//        else
//        {
//              this->sp.stop();
//              ROS_WARN("restart open serial port !!!");
//              ros::Duration(0.5).sleep();
//              this->sp.start(imu_port.c_str(),imu_port_baud);
//        }
    }

}

bool LPMS_NAV2::set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
    ROS_INFO("Set zero heading offset as current heading %2.3f" , this->current_heading);
    this->init_heading = this->current_heading;
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lpms_imu");

  LPMS_NAV2 imu_publisher;

  if (!imu_publisher.flag_port)
  {
      ROS_FATAL("shutting down");
//      ros::shutdown();
//      exit(0);
        return 0;
  }
  ros::Rate r(imu_publisher.imu_rate);
  while(ros::ok()) {
      imu_publisher.run();
      ros::spinOnce();
      r.sleep();
  }

  imu_publisher.sp.stop();
  return 0;
 
}

