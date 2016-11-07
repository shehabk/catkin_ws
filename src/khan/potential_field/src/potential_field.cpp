#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles


/*
 * Code Related to the vector calculation
 */
/*
 * Code Related to the vector calculation
 */

#define MAX(a,b) (a)>(b)?(a):(b)
#define MIN(a,b) (a)<(b)?(a):(b)
#define EPS 1e-10
// +0.0000000001
#define S(x)	((x)*(x))
#define Z(x)	(fabs(x) < EPS)

struct vector2d{
	double x,y ;
	vector2d(){x = 0 ; y = 0; };
	vector2d(double _x , double _y ){ x = _x ; y=_y ;}
	bool isZero(){
		return Z(x) && Z(y);
	}
	double mag(){
		return sqrt(mag2());
	}
	double mag2(){
		return S(x)+S(y);
	}
};

vector2d operator-(vector2d a){ return vector2d(-a.x , -a.y );}
vector2d operator-(vector2d a ,  vector2d b){ return vector2d(a.x-b.x , a.y-b.y) ;}
vector2d operator+(vector2d a ,  vector2d b){ return vector2d(a.x+b.x , a.y+b.y) ;}
vector2d operator/(vector2d a , double b){ return vector2d(a.x/b , a.y/b);}
vector2d operator*(double a , vector2d b){ return vector2d(a*b.x,a*b.y);}
double operator*(vector2d a , vector2d b){ return a.x*b.y - a.y*b.x ;}
double operator^(vector2d a , vector2d b){ return a.x*b.x + a.y*b.y ;}

double mag( vector2d a) { return sqrt( S(a.x) + S(a.y)) ;}
double dist( vector2d a , vector2d b) {return sqrt( S(a.x-b.x) + S(a.y-b.y)) ;}

vector2d unit( vector2d a ){ return a/mag(a) ;}
double proj( vector2d a , vector2d b ){ return a^unit(b) ;}
vector2d projv( vector2d a , vector2d b ){ vector2d ub = unit(b) ; return (a^ub)*ub ;}
vector2d rotate( vector2d a , double ang ){ vector2d b = vector2d(-a.y,a.x) ; return cos(ang)*a+sin(ang)*b ;}


double dotp(vector2d a, vector2d b){
	return a.x*b.x + a.y*b.y ;
};

double crossp(vector2d a, vector2d b){
	return a.x*b.y - a.y*b.x;
};

double angle(vector2d a, vector2d b){
	//if(fabs(a.mag2())<EPS||fabs(b.mag2())<EPS)return 0;
	double v= dotp(a,b) / (a.mag()*b.mag()) ;
	v=MIN(v,1);
	v=MAX(v,-1);
	return acos(v);
};


double getAngle(vector2d a , vector2d b ){
	double v = crossp(a, b);
	if(v<0) return -1*angle(a,b);
	else return angle(a,b);
};

/*
 *  Vector code ends.
 */

struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };
};


class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy) {

	// Initialize the local minima
	isLocalMinima = false;
	vector2d xAxis = vector2d(1 , 0 );
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++) {
      pose.push_back(Pose());
    }

	for (int i = 0; i < numRobots; i++) {
	  poseSubs.push_back(nh.subscribe("/base_pose_ground_truth", 1, \
		&Pose::poseCallback, &pose[i]));
	}
//    for (int i = 0; i < numRobots; i++) {
//      poseSubs.push_back(nh.subscribe("/robot_" + \
//        boost::lexical_cast<std::string>(i) + \
//        "/base_pose_ground_truth", 1, \
//        &Pose::poseCallback, &pose[i]));
//    }
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
	vector2d unitVector(1,0) ;
	double singleMag ;
	double angle ;
	repulsiveForce.clear();
	unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
	unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);

	for (int i = minIndex + 1; i < maxIndex; i++) {
		if((dSafe + epsilon < msg->ranges[i]) && (msg->ranges[i] < beta))
			singleMag = alpha / pow((msg->ranges[i] - dSafe), 2);
		else if((msg->ranges[i] < dSafe + epsilon))
			singleMag = alpha / pow(epsilon,2);
		else
			singleMag = 0;

		angle = (msg->angle_min + i * msg->angle_increment) + pose[ID].heading;
//		std::cout<<angle*180/4<<" "<<singleMag<<std::endl;
		repulsiveForce.push_back(singleMag*rotate(unitVector, angle));
	}
  };
  

  /*
   * Function to calculate the resultant force.
   */
  vector2d getAttractiveForce(){

      double attractiveX = (goalX - pose[ID].x);
      double attractiveY = (goalY - pose[ID].y);
      double attractiveMag = gamma * sqrt(pow((goalY - pose[ID].y),2) + pow((goalX - pose[ID].x),2));
      vector2d attractiveForce = attractiveMag * unit(vector2d(attractiveX, attractiveY));
      return attractiveForce ;

  }

  vector2d getRepulsiveForce(){
      vector2d repulsiveForceSum = vector2d(0,0) ;
      for ( int i = 0 ; i<repulsiveForce.size() ; i++ ){
    	  repulsiveForceSum = repulsiveForceSum + repulsiveForce[i] ;
      }
      return repulsiveForceSum;
  }

  vector2d getRandomForce(){
	  double randomX = (rand()%100)/100.0 ;
	  double randomY = (rand()%100)/100.0 ;
	  if(rand()%2) randomX*=-1;
	  if(rand()%2) randomY*=-1;
	  vector2d randomForce = vector2d(randomX , randomY );
	  randomForce = unit(randomForce);
	  std::cout << "RandomForce "<<randomForce.x<<" "<<randomForce.y <<std::endl ;
	  return randomForce;
  }

  bool isAtGoal(){
      double attractiveX = (goalX - pose[ID].x);
      double attractiveY = (goalY - pose[ID].y);
      vector2d goalDir = vector2d(attractiveX , attractiveY );
      if(goalDir.mag() < .02 ){
    	  return true ;
      }else {
    	  return false;
      }
  }

  void updatLocalMinima(){

	  vector2d currentPose = vector2d( pose[ID].x , pose[ID].y );
	  lastPositions.push_back(currentPose);

	  if(lastPositions.size()<=numOfLastToConsider){
		  return ;
	  }


	  double totalDistance = 0 ;

	  for ( int i=0 ; i < numOfLastToConsider ;i++){
		  totalDistance+= (lastPositions[i]-currentPose).mag();
	  }

	  totalDistance/= numOfLastToConsider;
	  lastPositions.erase(lastPositions.begin());

	  if(totalDistance < localMinThresh && !isAtGoal()&& !isLocalMinima){
		  isLocalMinima = true ;
		  minimaStartTime = ros::Time::now();
		  minimaDuration  = ros::Duration(15);
	  } else if((ros::Time::now() > minimaStartTime + minimaDuration) &&isLocalMinima){
		  isLocalMinima = false ;
	  }

	  std::cout<<"Average Distance: "<<totalDistance<<" Size "<<lastPositions.size()<<std::endl;

  }
  vector2d calculateResultantForce(){

	  vector2d attractiveForce = getAttractiveForce();
	  if(isLocalMinima){
		  vector2d randForce = 20*getRandomForce();
		  attractiveForce = 100*randForce ;
		  std::cout << "Is at Local Minima " <<std::endl ;

	  }

      vector2d repulsiveForceSum = getRepulsiveForce();
      vector2d resultantForce = attractiveForce-repulsiveForceSum;

      std::cout << "Pose: " << pose[ID].x << ", " << pose[ID].y << ", " << pose[ID].heading*180/M_PI << std::endl;
      std::cout<<"Attractive Force: "<<attractiveForce.mag()<<","<<getAngle(xAxis, attractiveForce)*180/M_PI<<std::endl;
      std::cout<<"Repulsive Force: "<<repulsiveForceSum.mag()<<","<<getAngle(xAxis, repulsiveForceSum)*180/M_PI<<std::endl;
      std::cout<<"Resultant Force: "<<resultantForce.mag()<<","<<getAngle(xAxis, resultantForce)*180/M_PI<<std::endl;
      std::cout<<" ----------- "<<std::endl;

      return resultantForce;
  }

  /*
   * My other methods gets here:
   */

  void makeMove( vector2d resultantForce ){

	  vector2d headingVector  = vector2d(cos(pose[ID].heading ), sin(pose[ID].heading));
      double diffAngle  = getAngle(headingVector, resultantForce);
      double diffLinear = proj(resultantForce, headingVector) ;
      std::cout << "DiffAngle: " << diffAngle*180/M_PI << "Linear: "<<diffLinear<<std::endl;
      std::cout << "HeadAngle: " << getAngle(xAxis, headingVector)*180/M_PI <<"Real angle "<<pose[ID].heading*180/M_PI<<std::endl;
      std::cout<<" ----------- "<<std::endl;
      double velAng = kpAng*(diffAngle);
      double velLin = kpLin*(diffLinear);
      velLin = MAX(velLin, velLinMin);
      velLin = MIN(velLin, velLinMax);
      move(velLin, velAng);

  }
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove demo code, compute potential function, actuate robot
    
      // Demo code: print each robot's pose
//      for (int i = 0; i < numRobots; i++) {
//        std::cout << std::endl;
//        std::cout << i << "        ";
//        std::cout << "Pose: " << pose[i].x << ", " << pose[i].y << ", " << pose[i].heading << std::endl;
//      }
      updatLocalMinima();
      vector2d resultantForce = calculateResultantForce();
      makeMove(resultantForce);

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };

  // Tunable motion controller parameters
  const static double MIN_SCAN_ANGLE_RAD = -90.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +90.0/180*M_PI;
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  const static float velLinMin = 0;
  const static float velLinMax = 20;
  const static float velAngMin = -M_PI/2.0;
  const static float velAngMax = M_PI/2.0;
  const static double alpha = 1;
  const static double gamma = 2;
  const static double beta = 4; //meters
  const static double epsilon = 0.05; //meters
  const static double dSafe = .5; //Meters
  const static float kpAng = .4;
  const static float kpLin = 1;
  const static double localMinThresh = 0.3 ;
  const static int numOfLastToConsider = 100;





protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  std::vector<ros::Subscriber> poseSubs; // List of subscribers to all robots' pose topics
  std::vector<Pose> pose; // List of pose objects for all robots
  int ID; // 0-indexed robot ID
  int numRobots; // Number of robots, positive value
  double goalX, goalY; // Coordinates of goal
  std::vector<vector2d> repulsiveForce;
  std::vector<vector2d> randomForce ;
  std::vector<vector2d> lastPositions;
  ros::Time minimaStartTime; // Start time of the rotation
  ros::Duration minimaDuration; // Duration of the rotation
  bool isLocalMinima ;
  vector2d xAxis ;
  int minimaCount;

};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  //ros::NodeHandle n("robot_" + std::string(argv[1])); // Create named handle "robot_#"
  ros::NodeHandle n;
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
