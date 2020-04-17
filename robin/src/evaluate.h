#include <NAOKinematics.h>
#include <KinematicsDefines.h>
#include <Eigen/Dense>
#include <alcommon/albroker.h>
#include <alproxies/almotionproxy.h>
#include <cmath>
#include <fstream>

#include "kinect.h"

using namespace KDeviceLists;
using namespace std;
using namespace AL;
using Eigen::Matrix3f;
using Eigen::MatrixXd;
using Eigen::Vector2f;
using Eigen::Rotation2Df;

class Evaluate {
   public:
      Evaluate();
      //Calculates the error of the human posture relative to Nao of the corresponding limb
      double evaluate(const int limbID, const bool logError);
      double getKinematicsAngle(const int angleID);
      double getAngleABC(double* a, double* b, double* c);
      vector<float> getJointAnglesList();     
      vector<float> getKinematicsCoordinates(const int jointID);
      void changeKinematicsAngle(const int angleID, const double amount);
      void setKinematicsAngle(const int angleID, const double amount);
      void setJoints(const vector<float>& robotJointAngles); 
      void prepareForward();
      void updateHumanSkeleton(const vector<Joint>& humanSkeleton);       
   private:
      vector<map<int, int> > humanJointToRobotJoint;
      NAOKinematics kinematics;
      vector<Joint> humanJointInfo;
      //Base joints of the limbs
      vector<int> baseHuman;
      vector<int> baseNao;      
      Matrix3f rx, ry, rz;      
      void getRotationMatrices();
      void getBalanceData();
      int timeStart;
      bool first;
};
