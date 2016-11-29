#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>


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
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("odom", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS ;
    commandPub.publish(msg) ;
  };
//http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
  void drawLine(int x1,
      int y1,
      int  x2,
      int  y2,
	  char value)
  {
	  //float  ratio  = canvas.rows / 50.0 ;
	  float ratio = 100.0/RESOLUTION ;
	  x1 = x1*ratio ;
	  y1 = y1*ratio ;
	  x2 = x2*ratio ;
	  y2 = y2*ratio ;

      int delta_x(x2 - x1);
      // if x1 == x2, then it does not matter what we set here
      signed char const ix((delta_x > 0) - (delta_x < 0));
      delta_x = std::abs(delta_x) << 1;

      int delta_y(y2 - y1);
      // if y1 == y2, then it does not matter what we set here
      signed char const iy((delta_y > 0) - (delta_y < 0));
      delta_y = std::abs(delta_y) << 1;

      plot(x1, y1,value);

      if (delta_x >= delta_y)
      {
          // error may go below zero
          int error(delta_y - (delta_x >> 1));

          while (x1 != x2)
          {
              if ((error >= 0) && (error || (ix > 0)))
              {
                  error -= delta_x;
                  y1 += iy;
              }
              // else do nothing

              error += delta_y;
              x1 += ix;

              plot(x1, y1,value );
          }
      }
      else
      {
          // error may go below zero
          int error(delta_x - (delta_y >> 1));

          while (y1 != y2)
          {
              if ((error >= 0) && (error || (iy > 0)))
              {
                  error -= delta_y;
                  x1 += ix;
              }
              // else do nothing

              error += delta_x;
              y1 += iy;

              plot(x1, y1,value );
          }
      }
  }


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
	  float ratio = 100.0/RESOLUTION ;

	  const std::vector<float> &allRange = msg->ranges ;
	  unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	  unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	  //ROS_INFO("Drawing line %u %u ",  minIndex ,  maxIndex  );
	  for (unsigned int i = 0 ; i < allRange.size(); i++) {
		  float cur_range = allRange[i];
		  //ROS_INFO("Drawing line %.3f",  cur_range );
		  if( cur_range > rangeMin){
			  float angle = heading + msg->angle_min + i* msg->angle_increment ;
			  float xprojection = cur_range*cos(angle);
			  float yprojection = cur_range*sin(angle);

			 // ROS_INFO("Drawing line  %.3f %.3f %.3f %.3f",x , y  ,x - yprojection , y + xprojection );

			  drawLine(x, y , x - yprojection, y + xprojection, CELL_FREE );

			  if(cur_range < rangeMax){
				  plot(floor((x-yprojection)*ratio), floor((y+xprojection)*ratio), CELL_OCCUPIED );
			  }

			  plot(floor(x*ratio), floor(y*ratio), CELL_ROBOT );

		  }
	  }
  };
  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Drawing line  %.3f %.3f %.3f",x , y  ,heading*180 / 3.1416);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock() ;
    canvas = cv::Scalar(CELL_UNKNOWN ) ;
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
     /*
      plot(x, y, rand() % 255); // Demo code: plot robot's current position on canvas
      plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
      plotImg(0, canvas.rows-1, CELL_UNKNOWN);
      plotImg(canvas.cols-1, 0, CELL_FREE);
      plotImg(canvas.cols-1, canvas.rows-1, CELL_ROBOT);
	 */
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      } else if( key == 'c'){
    	clearCanvas();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  void clearCanvas() {
    canvasMutex.lock();

    for( int i = 0 ; i< canvas.rows ; i++ )
    	for( int j = 0 ; j< canvas.cols ; j++ ){
    		canvas.at<char>(i, j) =  CELL_UNKNOWN ;
    	}
    canvasMutex.unlock();
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  const static double RESOLUTION = 5.0 ;

  const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;

  const static double rangeMax = 5 ;
  const static double rangeMin = .2 ;
  const static int SPIN_RATE_HZ = 30;
  

  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;



protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
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
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n, width, height); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
