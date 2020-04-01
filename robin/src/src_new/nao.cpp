#include "NAOKinematics.h"
#include "nao.h"
#include "stdio.h"
#include "math.h"
#include <cmath>

Nao::Nao(const string ip, const int port) {  
   motion = new ALMotionProxy(ip, port);
   motion->setCollisionProtectionEnabled("Arms", true);
   memory = new ALMemoryProxy(ip, port);
   posture = new ALRobotPostureProxy(ip, port);

   nextJoints = vector<float>(ANGLES);   
   nextJoints[0] = nextJoints[1] = 0;

   vector<float> joints = motion->getAngles("Joints", false);

   for (int i = 0; i < ANGLES; i++)
      targetJoints[i] = joints[i];   

   double lowerLim[] = {HeadYawLow, HeadPitchLow, 
                     LShoulderPitchLow, LShoulderRollLow, LElbowYawLow, LElbowRollLow, LWristYawLow, 
                     LHipYawPitchLow, LHipRollLow, LHipPitchLow, LKneePitchLow, LAnklePitchLow, LAnkleRollLow,
                     RHipYawPitchLow, RHipRollLow, RHipPitchLow, RKneePitchLow, RAnklePitchLow, RAnkleRollLow,
                     RShoulderPitchLow, RShoulderRollLow, RElbowYawLow, RElbowRollLow, RWristYawLow
                  };
   
   double upperLim[] = {HeadYawHigh, HeadPitchHigh, 
                     LShoulderPitchHigh, LShoulderRollHigh, LElbowYawHigh, LElbowRollHigh, LWristYawHigh, 
                     LHipYawPitchHigh, LHipRollHigh, LHipPitchHigh, LKneePitchHigh, LAnklePitchHigh, LAnkleRollHigh,
                     RHipYawPitchHigh, RHipRollHigh, RHipPitchHigh, RKneePitchHigh, RAnklePitchHigh, RAnkleRollHigh,
                     RShoulderPitchHigh, RShoulderRollHigh, RElbowYawHigh, RElbowRollHigh, RWristYawHigh
                  };

   jointsLowerLim = vector<double>(lowerLim, lowerLim + ANGLES);
   jointsUpperLim = vector<double>(upperLim, upperLim + ANGLES);

   humanJointInfo = vector<Joint>(ANGLES);
}//constructor

Nao::~Nao() {
   goToInitPost();
}

void Nao::goToInitPost() {
   posture->goToPosture("StandInit", 0.3);
   motion->wbFootState("Fixed", "Legs");
   posture->goToPosture("StandZero", 0.3);
}//goToInitPost

void Nao::balanceInit() {   
   motion->wbEnable(true);
   motion->wbFootState("Fixed", "Legs");
   motion->wbEnableBalanceConstraint(true, "Legs");
   currSupport = nextSupport = LEGS;
}//balanceInit

void Nao::updateHumanInfo(const vector<Joint>& humanSkeleton) {
   humanJointInfo = humanSkeleton;
   evaluator.updateHumanSkeleton(humanJointInfo);   
}//updateHumanInfo

void Nao::updateLoop() {   
   KVecDouble3 res;
   ALValue timeLists;
   vector<float> curr, prevCurr;
   double ratio;
   double maxVelocity;
   double multiplier;
   double dist;
   double error, prevError;
   double currError[LIMBS];
   vector<double> prevTargetJoints(ANGLES);   

   motion->setStiffnesses("Joints", 1.0);
   
   boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
   
   for (int i = 0; i < ANGLES; i++)
      timeLists.arrayPush(0.01f);  
   
   balanceInit();

   while (true) {
      if (currSupport == nextSupport) {
         dist = 0;
         for (int i = 2; i < ANGLES; i++)
            dist += fabs(prevTargetJoints[i] - targetJoints[i]);
         if (dist < 0.3) {
            cout << "NO MOVEMENT NEEDED" << endl;
            continue;
         }//if
      }//if
      prevError = INF;
      while (true) {
         updateSupport(); //checks whether the support mode has to be changed and acts accordingly

         curr = motion->getAngles("Joints", true); //the current angles of Nao
         error = 0;

         evaluator.setJoints(motion->getAngles("Joints", false));
         for (int i = 0; i < 4; i++) {
            currError[i] = evaluator.evaluate(i, true);     
            error += currError[i];
         }//for
                  
         if ((error > prevError && currError[0] < 2.0 && currError[3] < 2.0 &&
            (!(currSupport == LLEG && currError[2] > 2.0)) &&
            (!(currSupport == RLEG && currError[1] > 2.0)) &&
            (!(currSupport == LEGS && (currError[1] > 2.0 || currError[2] > 2.0)))) ||
            error != error) {
            cout << "CLOSE ENOUGH" << endl;
            break;
         }//if
     
         maxVelocity = 0.0f;      
         for (int i = 2; i < ANGLES; i++)
            maxVelocity = max(maxVelocity, fabs(targetJoints[i] - curr[i]) * 100);

         ratio = (currSupport == LEGS ? min(3.5 / maxVelocity, 1.0) : min(2.0 / maxVelocity, 1.0)); 
         
         for (int i = 0; i < LIMBS; i++) {
            multiplier = (i == leftLeg || i == rightLeg) ? 0.5 : 1.0;
            for (int j = 0; j < limbSize[i]; j++)    
               nextJoints[limbStart[i] + j] = (float)(multiplier * ratio * (targetJoints[limbStart[i] + j] - curr[limbStart[i] + j]));     
         }//for

         if (currSupport != LEGS)
            setFreeLegAnkleAngles(nextJoints, curr); //try to keep foot parallel to the ground as much as possible by adjusting ankle angles   
                           
         for (int i = 2; i < ANGLES; i++) {         
            if ((i < limbStart[1] || i >= limbStart[3]) && fabs(nextJoints[i]) < 0.003)    
               nextJoints[i] = 0.0;
         }//for
       
         prevError = error;

         try {
            motion->angleInterpolation("Joints", nextJoints, timeLists, false);
         }
         catch (AL::ALError& e) {
            cout << e.toString() << endl;
         }  
      }//while
      for (int i = 2; i < ANGLES; i++)
         prevTargetJoints[i] = targetJoints[i];
   }//while  
}//updateLoop

void Nao::setFreeLegAnkleAngles(vector<float>& nextJoints, const vector<float>& curr) {
   vector<float> coordinatesFL, coordinatesRL, coordinatesRR;
   double angle;
   if (currSupport == LLEG) {
      coordinatesFL = motion->getPosition("RFoot/FSR/FrontLeft", 1, true);
      coordinatesRL = motion->getPosition("RFoot/FSR/RearLeft", 1, true);   
      coordinatesRR = motion->getPosition("RFoot/FSR/RearRight", 1, true);
      angle = atan(fabs(coordinatesFL[2] - coordinatesRL[2]) / 0.1050);
      if (coordinatesFL[2] > coordinatesRL[2])
         angle *= -1;   
      nextJoints[RAnklePitch] = (float)max(min(-angle, RAnklePitchHigh), RAnklePitchLow);

      angle = (float)atan(fabs(coordinatesRR[2] - coordinatesRL[2]) / 0.0490);
      if (coordinatesRR[2] > coordinatesRL[2])
         angle *= -1;
      nextJoints[RAnkleRoll] = (float)max(min(-angle, RAnkleRollHigh), RAnkleRollLow);
   }//if
   else if (currSupport == RLEG) {
      coordinatesFL = motion->getPosition("LFoot/FSR/FrontLeft", 1, true);
      coordinatesRL = motion->getPosition("LFoot/FSR/RearLeft", 1, true);  
      coordinatesRR = motion->getPosition("LFoot/FSR/RearRight", 1, true);

      angle = atan(fabs(coordinatesFL[2] - coordinatesRL[2]) / 0.1050);
      if (coordinatesFL[2] > coordinatesRL[2])
         angle *= -1;
      nextJoints[LAnklePitch] = (float)max(min(-angle, LAnklePitchHigh), LAnklePitchLow);

      angle = atan(fabs(coordinatesRL[2] - coordinatesRR[2]) / 0.0490);
      if (coordinatesRR[2] > coordinatesRL[2])
         angle *= -1;
      nextJoints[LAnkleRoll] = (float)max(min(-angle, LAnkleRollHigh), LAnkleRollLow);
   }//else if
}//setFreeLegAnkleAngles

bool Nao::footCollision() {
   vector<float> coordinatesFL, coordinatesRL;   
   vector<point_xy> points1, points2;
   boost::geometry::model::polygon<point_xy> polygon1, polygon2; 
   Vector2f point, rotated;
   Rotation2Df rot;
   float angle;

   vector<vector<float> > footCoordinates = vector<vector<float> >(4, vector<float>(2));
   //LEFT UPPER
   footCoordinates[0][0] = (float)0.0;
   footCoordinates[0][1] = (float)16.22;

   //RIGHT UPPER
   footCoordinates[1][0] = (float)9.45;
   footCoordinates[1][1] = (float)16.22;

   //RIGHT BOTTOM
   footCoordinates[2][0] = (float)9.45;
   footCoordinates[2][1] = (float)0.0;

   ALValue WL = memory->getData("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
   ALValue WR = memory->getData("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");  

   if ((float)WL > 0 && (float)WR > 0) //both feet are already on the ground
      return false;

   cout << (float)WL << " " << (float)WR << endl;
   
   cout << "LEFT" << endl;
   //LEFT FOOT
   coordinatesFL = motion->getPosition("LFoot/FSR/FrontLeft", 0, false);
   coordinatesRL = motion->getPosition("LFoot/FSR/RearLeft", 0, false);   
   
   angle = -(float)(atan2(coordinatesFL[0] - coordinatesRL[0], coordinatesFL[1] - coordinatesRL[1]) - 0.5 * PI);
   rot = Rotation2Df(angle);
   points1.push_back(point_xy(-100.0 * coordinatesRL[1], 100.0 * coordinatesRL[0])); //the coordinates of the left bottom corner
   for (int i = 0; i < 3; i++) {
      point(0, 0) = footCoordinates[i][0];
      point(1, 0) = footCoordinates[i][1];     
      rotated = rot.toRotationMatrix() * point;
      points1.push_back(point_xy(rotated(0, 0) - 100.0 * coordinatesRL[1], rotated(1, 0) + 100.0 * coordinatesRL[0]));
      cout << rotated(0, 0) - 100.0 * coordinatesRL[1] << " " << rotated(1, 0) + 100.0 * coordinatesRL[0] << endl;
   }//for
   boost::geometry::assign_points(polygon1, points1);

   cout << "RIGHT" << endl;
   //RIGHT FOOT
   coordinatesFL = motion->getPosition("RFoot/FSR/FrontLeft", 0, false);
   coordinatesRL = motion->getPosition("RFoot/FSR/RearLeft", 0, false);
   angle = -(float)(atan2(coordinatesFL[0] - coordinatesRL[0], coordinatesFL[1] - coordinatesRL[1]) - 0.5 * PI);
   rot = Rotation2Df(angle);
   points2.push_back(point_xy(-100.0 * coordinatesRL[1], 100.0 * coordinatesRL[0])); //the coordinates of the left bottom corner
   for (int i = 0; i < 3; i++) {
      point(0, 0) = footCoordinates[i][0];
      point(1, 0) = footCoordinates[i][1];
      rotated = rot.toRotationMatrix() * point;
      points2.push_back(point_xy(rotated(0, 0) - 100.0 * coordinatesRL[1], rotated(1, 0) + 100.0 * coordinatesRL[0]));
      cout << rotated(0, 0) - 100.0 * coordinatesRL[1] << " " << rotated(1, 0) + 100.0 * coordinatesRL[0] << endl;
   }//for
   boost::geometry::assign_points(polygon2, points2);

   if (boost::geometry::intersects(polygon1, polygon2)) 
      cout << "Feet collide. Can't put free foot back on the ground." << endl;
   return boost::geometry::intersects(polygon1, polygon2); //returns whether support polygons of feet intersect
}//footCollision

bool Nao::switchToLegs() {
   vector<float> coordinatesL = motion->getPosition("LLeg", 1, false);
   vector<float> coordinatesR = motion->getPosition("RLeg", 1, false);   

   if (fabs(coordinatesL[2] - coordinatesR[2]) > 0.04) {//Foot has to be 4.0 centimeters or less off the ground
      if (rand() % 15 == 0)
         cout << "Putting foot back on the ground." << endl;
      return false;
   }//if
   if (footCollision()) //Feet polygons may not collide when free foot is put back on the ground
      return false;

   cout << "Switching to LEGS support mode." << endl;

   motion->wbFootState("Fixed", "Legs");
   try {
      motion->wbGoToBalance("Legs", 3.0f);   
   }
   catch (AL::ALError& e) {
      cout << e.toString() << endl;
   }             
   motion->wbEnableBalanceConstraint(true, "Legs");
   currSupport = LEGS;
   return true;
}//switchToLegs

bool Nao::switchToLLeg() {
   cout << "Switching to LLEG support mode." << endl;
   try {
      motion->wbGoToBalance("LLeg", 3.0f);   
   }
   catch (AL::ALError& e) {
      cout << e.toString() << endl;
   }
   motion->wbFootState("Fixed", "LLeg");
   motion->wbFootState("Free", "RLeg");    
   motion->wbEnableBalanceConstraint(true, "LLeg");
   currSupport = LLEG;
   return true;
}//switchToLLeg

bool Nao::switchToRLeg() {
   cout << "Switching to RLEG support mode." << endl;
   try {
      motion->wbGoToBalance("RLeg", 3.0f);   
   }
   catch (AL::ALError& e) {
      cout << e.toString() << endl;
   }
   motion->wbFootState("Fixed", "RLeg");
   motion->wbFootState("Free", "LLeg");
   motion->wbEnableBalanceConstraint(true, "RLeg");
   currSupport = RLEG;
   return true;
}//switchToRLeg

bool Nao::updateSupport() {
   if (currSupport != nextSupport) {
      if (currSupport == LLEG || currSupport == RLEG)
         return switchToLegs();
      else if (nextSupport == LLEG)
         return switchToLLeg();
      else
         return switchToRLeg();
   }//if   
   return false;
}//updateSupport

void Nao::setTargetAngles(const vector<float>& angles) {
   for (int i = 2; i < ANGLES; i++)
      targetJoints[i] = angles[i];
}//setTargetAngles

bool Nao::supportmodeChangeNeeded() {
   return (currSupport != nextSupport);
}

void Nao::setNextSupport(const supportMode s) {
   nextSupport = s;
}//setNextSupport

vector<float> Nao::getJointAngles() {
   return motion->getAngles("Joints", false);
}//getJoints