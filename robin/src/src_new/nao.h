#include <iostream>
#include "evaluate.h"
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alerror/alerror.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono/chrono.hpp>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry.hpp>

using namespace KDeviceLists;
using namespace std;
using namespace AL;
using Eigen::Matrix3f;
using Eigen::MatrixXd;
using Eigen::Vector2f;
using Eigen::Rotation2Df;

typedef boost::geometry::model::d2::point_xy<double> point_xy;

class Nao {
	public:
      //Variables
      Nao() {motion = nullptr; jointsBusy = nullptr;}
      Nao(const string ip, const int port);
      ~Nao();

      vector<double> jointsLowerLim;
      vector<double> jointsUpperLim;

      vector<float> getJointAngles();
      vector<Joint> humanJointInfo;
         
      //Functions   
      bool supportmodeChangeNeeded();      
      void setTargetAngles(const vector<float>& angles);
      void setNextSupport(const supportMode s);
      void updateLoop();
      void updateHumanInfo(const vector<Joint>& humanSkeleton);
	private:    
      vector<float> nextJoints;
      double targetJoints[24];      

      supportMode currSupport, nextSupport; 
      boost::atomic<double*>* jointsBusy;

      Evaluate evaluator;

      ALMotionProxy* motion;
      ALMemoryProxy* memory;
      ALRobotPostureProxy* posture;

      bool switchToLegs();
      bool switchToLLeg();
      bool switchToRLeg();
      
      void setFreeLegAnkleAngles(vector<float>& nextJoints, const vector<float>& curr);
      bool footCollision();
      void balanceInit();
      void goToInitPost();
      bool updateSupport();
      void startTimedLoop();
};