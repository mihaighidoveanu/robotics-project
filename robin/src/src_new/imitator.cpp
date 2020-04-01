#include <iostream>
#include <Eigen/Dense>
#include "imitator.h"
#include <alproxies/almotionproxy.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <fstream>

Imitator::Imitator(char* ip, const int port) {
   human = HumanSkeleton();
   nao = Nao(string(ip), port);

   boost::thread imitatorThread(boost::bind(&Nao::updateLoop, &nao));     
   currHumanJointInfo = prevHumanJointInfo = humanJointInfo = vector<Joint>(JointType_Count);
        
   //Initialize Jacobian matrices
   Jacobian = vector<MatrixXd>(LIMBS);
   for (int i = 0; i < LIMBS; i++)
      Jacobian[i] = MatrixXd(1, limbSize[i]); 
   currError = vector<double>(LIMBS);
   prevIKAngles = vector<double>(ANGLES);

   supportCounter = 0;
}//constructor

void Imitator::fillJacobian(const int limbID) {
   if (!(limbID >= 0 && limbID < LIMBS)) {
      cerr << "Unvalid limb ID passed to the Jacobian calculation function." << endl;
      return;
   }//if

   double currentValue;
   double newError;

   for (int j = 0; j < limbSize[limbID]; j++) {
      currentValue = evaluator.getKinematicsAngle(limbStart[limbID] + j); //Store current value
      evaluator.changeKinematicsAngle(limbStart[limbID] + j, ANGLEDIFF * TORADIAN); //Increase current joint angle by ANGLEDIFF degrees
      newError = evaluator.evaluate(limbID, false); //Get the new error after joint angle increasement
      evaluator.setKinematicsAngle(limbStart[limbID] + j, currentValue); //Restore joint angle value
      Jacobian[limbID](0, j) = newError - currError[limbID]; //Store effect of joint angle increasement on the error in Jacobian matrix
   }//for
}//fillJacobian

void Imitator::inverseKinematics(const int limbID, const int type, double prevIKError) {
   if (!(limbID >= 0 && limbID < LIMBS)) {
      cerr << "Unvalid limb ID passed to the IK function." << endl;
      return;
   }//if 

   double lambda = LAMBDASTART;
   double newError;
   double min, max;

   //Contains the angle values of the limb before adjustment
   vector<double> currentValuesC(limbSize[limbID]);
   
   //Contains the amount the angles will be changed
   MatrixXd Ocurr(limbSize[limbID], 1);
   
   for (int i = 0; i < MAX_IK_ITERATIONS; i++) {
      //Iterative update
      fillJacobian(limbID);

      //LM equation
      Ocurr = (Jacobian[limbID].transpose() * Jacobian[limbID] + lambda * 
               MatrixXd::Identity(limbSize[limbID], limbSize[limbID])).inverse() * 
               Jacobian[limbID].transpose() * -currError[limbID];

      for (int j = 0; j < limbSize[limbID]; j++) {   
         currentValuesC[j] = evaluator.getKinematicsAngle(limbStart[limbID] + j);               
         evaluator.changeKinematicsAngle(limbStart[limbID] + j, TORADIAN * Ocurr(j, 0));         
      }//for

      newError = evaluator.evaluate(limbID, false); //new error after joint angle adjustment is calculated
      if (newError - currError[limbID] > 0.0 && newError - currError[limbID] < EPS)
         break;
      else if (newError < currError[limbID]) { //ACCEPT angle change
         lambda /= 2.0;
         currError[limbID] = newError;         
      }//if
      else { //ABONDON angle change and restore angle values  
         for (int j = 0; j < limbSize[limbID]; j++)
            evaluator.setKinematicsAngle(limbStart[limbID] + j, currentValuesC[j]);  
         lambda *= 2.0;
      }//else
   }//for   
   if (type == 0)
      newError = min(newError, currError[limbID]);

   //If the IK has been started for a 2nd time, but the new error is greater,
   //use the best obtained angles instead
   if (type > 0 && prevIKError < newError) {
      for (int i = 0; i < limbSize[limbID]; i++)
         evaluator.setKinematicsAngle(limbStart[limbID] + i, prevIKAngles[limbStart[limbID] + i]);
   }//if   
   
   //If the error is still greater than 0.5, try to obtain a lower error by running IK starting with different values
   if (newError > 0.5 && type < 2) {
      if (type == 0) {
         for (int i = 0; i < limbSize[limbID]; i++) {         
            prevIKAngles[limbStart[limbID] + i] = evaluator.getKinematicsAngle(limbStart[limbID] + i);
            evaluator.setKinematicsAngle(limbStart[limbID] + i, 0.0); //the zero posture
         }//for
         return inverseKinematics(limbID, 1, newError);
      }//if
      else if (type == 1) {
         if (newError < prevIKError)
            for (int i = 0; i < limbSize[limbID]; i++)
               prevIKAngles[limbStart[limbID] + i] = evaluator.getKinematicsAngle(limbStart[limbID] + i);
         for (int i = 0; i < limbSize[limbID]; i++) {
            min = nao.jointsLowerLim[limbStart[limbID] + i];
            max = nao.jointsUpperLim[limbStart[limbID] + i];
            evaluator.setKinematicsAngle(limbStart[limbID] + i, (max - min) * ((double)rand() / (double)RAND_MAX) + min); //random posture           
         }//for
         return inverseKinematics(limbID, 2, min(prevIKError, newError));
      }//else if      
   }//if         
}//inverseKinematics

void Imitator::setNextSupportMode() {   
   double leftAnkleHeight = humanJointInfo[JointType_AnkleLeft].Position.Y;
   double rightAnkleHeight = humanJointInfo[JointType_AnkleRight].Position.Y;  
   if (leftAnkleHeight - rightAnkleHeight > 0.15) {
      if (supportCounter < 0)
         supportCounter = 0;
      if (supportCounter++ == 3)
         nao.setNextSupport(RLEG);
   }//if
   else if (rightAnkleHeight - leftAnkleHeight > 0.15) {
      if (supportCounter > 0)
         supportCounter = 0;
      if (supportCounter-- == -3)
         nao.setNextSupport(LLEG);
   }//else if
   else {
      supportCounter = 0;
      nao.setNextSupport(LEGS);
   }//else
}//setNextSupportMode

double Imitator::skeletonDist(const vector<Joint>& prev, const vector<Joint>& curr) {
   double error = 0.0;
   Matrix3f prevC, currC; //Human joint, Nao joint

   prevC.setZero(3, 3);
   currC.setZero(3, 3);
   for (int i = 0; i < JointType_Count; i++) {
      prevC(0, 0) = prev[i].Position.X;
      prevC(1, 0) = prev[i].Position.Y;
      prevC(2, 0) = prev[i].Position.Z;

      currC(0, 0) = curr[i].Position.X;
      currC(1, 0) = curr[i].Position.Y;
      currC(2, 0) = curr[i].Position.Z;

      error += ((1 / prevC.norm()) * prevC - (1 / currC.norm()) * currC).norm();
   }//for
   if (error != error)
      return INF - 1;
   return error;
}//skeletonDist

void Imitator::imitate() {
   double closestCoordinate;
   double bestDistSoFar, currDist;
   bool first = true;

   srand(time(0));
   evaluator.setJoints(nao.getJointAngles()); 
   while (true) {
      //boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time(); 
      //GET ANGLES WITH IK (LM) TO MINIMIZE CURRENT ERROR

      if (!first) {    
         prevHumanJointInfo = humanJointInfo;
         bestDistSoFar = INF;
         for (int i = 0; i < 1; i++) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / 30.0));
            while (!human.getSkeletonCoordinates(currHumanJointInfo));
            currDist = skeletonDist(currHumanJointInfo, prevHumanJointInfo);
            
            if (currDist < bestDistSoFar) {      
               bestDistSoFar = currDist;
               humanJointInfo = currHumanJointInfo;
            }//if
         }//for
      }//if
      else {
         while (!human.getSkeletonCoordinates(humanJointInfo));
         first = false;
      }//else 

      closestCoordinate = INF;
      for (int i = 0; i < JointType_Count; i++)
         closestCoordinate = min(closestCoordinate, humanJointInfo[i].Position.Z);
      if (closestCoordinate < 1.0) {
         cout << "FRAME SKIPPED, SKELETON TOO CLOSE" << endl;
         continue;      
      }//if

      setNextSupportMode();    
      evaluator.updateHumanSkeleton(humanJointInfo);
           
      for (int i = 0; i < LIMBS; i++) {
         currError[i] = evaluator.evaluate(i, false);        
         inverseKinematics(i, 0, currError[i]);
      }//for
      //boost::posix_time::ptime tick = boost::posix_time::microsec_clock::local_time();
      //boost::posix_time::time_duration diff = tick - now;  
      
      nao.setTargetAngles(evaluator.getJointAnglesList());
      evaluator.setJoints(nao.getJointAngles());
      nao.updateHumanInfo(humanJointInfo);     
   }//while
}//imitate