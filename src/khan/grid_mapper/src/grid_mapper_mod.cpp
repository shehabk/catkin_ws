// src/grid_mapper.cpp

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>  // Needed for rand()
#include <ctime>    // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

// Includes

#include <algorithm>  // For std::swap
#include <cmath>      // For fabs




using namespace boost::posix_time;


class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
      canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("scan", 1,
        &GridMapper::laserCallback, this);

    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("odom", 1,
        &GridMapper::poseCallback, this);

    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas",
        CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };


  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time())
        + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };


  // Update grayscale intensity on canvas pixel (x, y)
  // (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  void plotWithResolution(double x, double y, char value) {
      plot((int)(x / RESOLUTION_M), (int)(y / RESOLUTION_M), value);
  }

  // Line Drawing Method

    // Bresenham's line algorithm (in robot coordinate frame)
    void plotLineWithResolution(double x0, double y0, double x1, double y1,
            char value) {
      x0 /= RESOLUTION_M;
      y0 /= RESOLUTION_M;
      x1 /= RESOLUTION_M;
      y1 /= RESOLUTION_M;
      bool steep = (y1 - y0) > (x1 - x0);
      if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
      }
      if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
      }
      double error = 0;
      double derror = fabs((y1 - y0) / (x1 - x0));
      double yinc = (y1 > y0) ? 1 : -1;
      int y = y0;
      for (int x = x0; x <= x1; ++x) {
        if (steep) { plot(y, x, value); } else { plot(x, y, value); }
        error += derror;
        while (error >= 0.5) {
          if (steep) { plot(y, x, value); } else { plot(x, y, value); }
          y += yinc;
          error -= 1.0;
        }
      }
    };



  // Update grayscale intensity on canvas pixel (x, y)
  // (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg;
          // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Laser Callback

    const std::vector<float> &ranges = msg->ranges;
    //size_t i = ranges.size() / 2;
    for (size_t i = 0; i < ranges.size(); ++i) {
      float m = ranges[i];
      if (m > msg->range_min) {
        double theta = heading + msg->angle_min + i*msg->angle_increment + M_PI/2;
        double lx = x + m*cos(theta);
        double ly = y + m*sin(theta);
        //ROS_INFO("Drawing line %.3f, %.3f, %.3f, %.3f", x, y, lx, ly);
        if (fabs(lx/x - 1.0) > 1.0/256.0 && fabs(x/lx - 1.0) > 1.0/256.0
                && fabs(ly/y - 1.0) > 1.0/256.0
                && fabs(y/ly - 1.0) > 1.0/256.0) {
            plotLineWithResolution(x, y, lx, ly, CELL_FREE);
        }
        if (m < msg->range_max) {
          plotWithResolution(lx, ly, CELL_OCCUPIED);
        }
      }
    }


  };


  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
  };


  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;

    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

      // Pre-Image-Display

      plotWithResolution(x, y, CELL_ROBOT);



      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);

      // Post-Image-Display

      plotWithResolution(x, y, CELL_FREE);



      ros::spinOnce();
            // Need to call this function often to allow ROS to process
            // incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ);
            // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }

    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 0.5;
  const static double ROTATE_SPEED_RADPS = M_PI/4;

  const static int SPIN_RATE_HZ = 30;

  const static char CELL_OCCUPIED = 0 ;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;

  const static double RESOLUTION_M = 0.075 ;


protected:
  ros::Publisher commandPub;
        // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub;
        // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub;
        // Subscriber to the current robot's ground truth pose topic

  double x;       // in simulated Stage units, + = East/right
  double y;       // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

  cv::Mat canvas;           // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;

  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]"
        << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n;  // Create default handle
  GridMapper robbie(n, width, height);  // Create new grid mapper object
  robbie.spin();      // Execute FSM loop

  return EXIT_SUCCESS;
};

