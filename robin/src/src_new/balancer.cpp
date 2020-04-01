#include "balancer.h"

Balancer::Balancer() {   
   rotateIDs = vector<vector<int> >(3, vector<int>());
   Jacobian = vector<MatrixXd>(3);

   rotateIDs[LLEG] = rotateIDs[RLEG] = vector<int>(2);
   rotateIDs[LEGS] = vector<int>(LEG_SIZE);

   rotateIDs[LLEG][0] = Rotations::LAnklePitch;
   rotateIDs[LLEG][1] = Rotations::LAnkleRoll;

   rotateIDs[RLEG][0] = Rotations::RAnklePitch;
   rotateIDs[RLEG][1] = Rotations::RAnkleRoll;

   for (int i = 0; i < 3; i++)
      Jacobian[i] = MatrixXd(1, (int)rotateIDs[i].size());  
}//constructor

double Balancer::singleSupportEvaluate(supportMode support) {
   NAOKinematics::kmatTable k1;
   if (support == LLEG) 
      k1 = kinematics.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_LEG);
   else
      k1 = kinematics.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG);

   NAOKinematics::kmatTable k2 = kinematics.getForwardEffector((NAOKinematics::Effectors)CHAIN_HEAD);
   KVecDouble3 c;
   Matrix3f COM, unitY, res;
   vector<float> ankleCoordinates;
   vector<float> coordinates = vector<float>(3);

   COM.setZero(3, 3);
   res.setZero(3, 3);
      
   c = kinematics.calculateCenterOfMass();
   
   while (k1(0, 0) != k1(0, 0)) {
      cout << "Forward effector error." << endl;
      return 10.0;
   }//while

   for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
         k2(i, j) = 0.0f;

   k1 = k1.fast_invert();
      
   k2(0, 3) = c(0);
   k2(1, 3) = c(1);
   k2(2, 3) = c(2);

   k1 = k1 * k2;

   COM(0, 0) = k1(1, 3);
   COM(1, 0) = k1(2, 3);
   COM(2, 0) = k1(0, 3);

   res = (1 / COM.norm()) * COM;
   res(1, 0) = 0;

   return pow(6.0 * res.norm(), 2);
}//singleSupportEvaluate

void Balancer::singleFillJacobian(supportMode support) {   
   double currentValue;
   double newError;
   const int rotations = (int)rotateIDs[support].size();  
   for (int j = 0; j < rotations; j++) {      
      currentValue = getKinematicsAngle(rotateIDs[support][j]); //Store current value
      changeKinematicsAngle(rotateIDs[support][j], ANGLEDIFF * TORADIAN); //Increase current joint angle by ANGLEDIFF degrees
      newError = singleSupportEvaluate(support); //Get the new error after joint angle increasement

      setKinematicsAngle(rotateIDs[support][j], currentValue); //Restore joint angle value
      Jacobian[support](0, j) = newError - currError; //Store effect of joint angle increasement on the error in Jacobian matrix      
   }//for     
}//fillJacobian

double Balancer::singleSupportBalance(supportMode support, vector<float>& angles) {
   //Adjust the ankle angles given to DCM such that the COM will be in a stable position.
   const int rotations = (int)rotateIDs[support].size();

   MatrixXd Ocurr(rotations, 1);
   double newError;
   double lambda = LAMBDASTART;
   
   //Contains the angle values of the limb before adjustment
   vector<double> currentValuesC(ANGLES); 
   for (int j = 0; j < ANGLES; j++)
      setKinematicsAngle(j, angles[j]);

   currError = singleSupportEvaluate(support);
 
   for (int i = 0; i < 50; i++) {
      //Iterative update
      singleFillJacobian(support);
      //LM equation
      Ocurr = (Jacobian[support].transpose() * Jacobian[support] + lambda * 
               MatrixXd::Identity(rotations, rotations)).inverse() * 
               Jacobian[support].transpose() * -currError;

      for (int j = 0; j < rotations; j++) {   
         currentValuesC[rotateIDs[support][j]] = getKinematicsAngle(rotateIDs[support][j]);               
         changeKinematicsAngle(rotateIDs[support][j], TORADIAN * Ocurr(j, 0));         
      }//for
      newError = singleSupportEvaluate(support); //new error after joint angle adjustment is calculated
      if (newError < currError) { //ACCEPT angle change
         lambda /= 2.0;
         currError = newError;         
      }//if
      else { //ABONDON angle change and restore angle values  
         for (int j = 0; j < rotations; j++)
            setKinematicsAngle(rotateIDs[support][j], currentValuesC[rotateIDs[support][j]]);  
         lambda *= 2.0;
      }//else
   }//for
   for (int j = 0; j < rotations; j++) 
      angles[rotateIDs[support][j]] = getKinematicsAngle(rotateIDs[support][j]); 
   return newError;
}//singleSupportBalance

double Balancer::getKinematicsAngle(const int angleID) {
   return kinematics.getJoint(angleID);
}//getKinematicsAngle

void Balancer::changeKinematicsAngle(const int angleID, const double amount) {
   kinematics.changeJoint(angleID, amount);
}//changeKinematicsAngle

void Balancer::setKinematicsAngle(const int angleID, const double amount) {   
   kinematics.setJoint(angleID, amount);
}//setKinematicsAngle

vector<float> Balancer::getKinematicsCoordinates(const int jointID) {
   NAOKinematics::kmatTable k = kinematics.getForwardEffector((NAOKinematics::Effectors)jointID);

   vector<float> coordinates = vector<float>(3);
   coordinates[0] = k(1, 3);
   coordinates[1] = k(2, 3);
   coordinates[2] = k(0, 3);

   return coordinates;
}//getKinematicsCoordinates