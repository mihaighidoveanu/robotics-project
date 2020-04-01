#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <NAOKinematics.h>
#include <KinematicsDefines.h>
#include <alerror/alerror.h>

using namespace std;
using namespace KDeviceLists;
using namespace AL;

using Eigen::Matrix3f;
using Eigen::MatrixXd;

class Balancer {
   public:
      //Variables
      Balancer();
      ~Balancer() {}
      double singleSupportBalance(supportMode support, vector<float>& angles);      
	private:
      NAOKinematics kinematics;
      vector<MatrixXd> Jacobian;
      double currError;
            
      void singleFillJacobian(supportMode support);
      double singleSupportEvaluate(supportMode support);

      vector<vector<int> > rotateIDs;

      double getKinematicsAngle(const int angleID);    
      void changeKinematicsAngle(const int angleID, const double amount);
      void setKinematicsAngle(const int angleID, const double amount);
      vector<float> getKinematicsCoordinates(const int jointID);
};

