#include <map>
#include <Eigen/Dense>
#include "humanSkeleton.h"
#include "nao.h"
#include <boost/atomic.hpp>
#include <kinect.h>
 
using namespace AL;
using namespace KDeviceLists;
using namespace std;
using Eigen::Matrix3f;
using Eigen::MatrixXd;

class Imitator {
   public:
      Imitator(char* ip, const int port);
      void imitate();
   private:
      //The human skeleton
      HumanSkeleton human;
      //Contains the coordinates of all of the human joints
      vector<Joint> humanJointInfo, currHumanJointInfo, prevHumanJointInfo;

      //The Nao robot
      Nao nao;
      Evaluate evaluator;

      //Rotation matrices 

      int supportCounter;
      
      //Base joints of the limbs
      vector<int> baseHuman;
      vector<int> baseNao;

      //Error of human posture relative to Nao of corresponding limb, calculated by the evaluate function
      vector<double> currError; 

      vector<MatrixXd> Jacobian;
      vector<double> prevIKAngles;
      //Angle values obtained by IK, balance maintenance and collision avoidance

      void setNextSupportMode();

      //Maps human joints to Nao joints (per limb), and stores the mapping in humanJointToRobotJoint
      void mapJoints();
      //Calculates the Jacobian matrix geometrically
      void fillJacobian(const int limbID);
      //IK kinematics function that minimizes the error by using the Levenberg-Marquardt algorithm
      void inverseKinematics(const int limbID, const int type, double prevIKError);
      
      double skeletonDist(const vector<Joint>& prev, const vector<Joint>& curr);
};
