#include "evaluate.h"

#include "stdio.h"
#include "math.h"

//ofstream out1("time.txt");
ofstream out2("larm.txt");
ofstream out3("rarm.txt");
ofstream out4("lknee.txt");
ofstream out5("rknee.txt");
ofstream out6("lelbow.txt");
ofstream out7("relbow.txt");
ofstream out8("lankle.txt");
ofstream out9("rankle.txt");

ofstream out13("test.txt");
ofstream out14("time2.txt");

Evaluate::Evaluate() {
   humanJointToRobotJoint = vector<map<int, int> >(LIMBS);
   humanJointInfo = vector<Joint>(JointType_Count);

   baseHuman = vector<int>(LIMBS);
   baseHuman[leftArm] = JointType_ShoulderLeft;
   baseHuman[leftLeg] = JointType_HipLeft;
   baseHuman[rightLeg] = JointType_HipRight;
   baseHuman[rightArm] = JointType_ShoulderRight;

   baseNao = vector<int>(LIMBS);
   baseNao[leftArm] = CHAIN_L_SHOULDER;
   baseNao[leftLeg] = CHAIN_L_HIP;
   baseNao[rightLeg] = CHAIN_R_HIP;
   baseNao[rightArm] = CHAIN_R_SHOULDER;
   //humanJointToRobotJoint[LEFT_ARM][JointType_ShoulderLeft]    = CHAIN_L_SHOULDER; //LIMB L_ARM
   humanJointToRobotJoint[LEFT_ARM][JointType_ElbowLeft]       = CHAIN_L_ELBOW;
   humanJointToRobotJoint[LEFT_ARM][JointType_HandLeft]        = CHAIN_L_ARM;
   
   //humanJointToRobotJoint[LEFT_LEG][JointType_HipLeft]         = ChainsNames::CHAIN_L_HIP; //LIMB L_LEG
   humanJointToRobotJoint[LEFT_LEG][JointType_KneeLeft]        = CHAIN_L_KNEE;
   humanJointToRobotJoint[LEFT_LEG][JointType_AnkleLeft]       = CHAIN_L_ANKLE;

   //humanJointToRobotJoint[RIGHT_LEG][JointType_HipRight]       = CHAIN_R_HIP; //LIMB R_LEG
   humanJointToRobotJoint[RIGHT_LEG][JointType_KneeRight]      = CHAIN_R_KNEE;
   humanJointToRobotJoint[RIGHT_LEG][JointType_AnkleRight]     = CHAIN_R_ANKLE;

   //humanJointToRobotJoint[RIGHT_ARM][JointType_ShoulderRight]  = CHAIN_R_SHOULDER; //LIMB R_ARM
   humanJointToRobotJoint[RIGHT_ARM][JointType_ElbowRight]     = CHAIN_R_ELBOW;
   humanJointToRobotJoint[RIGHT_ARM][JointType_HandRight]      = CHAIN_R_ARM;

   first = true;   
}//constructor

void Evaluate::prepareForward() {
   kinematics.prepareForward();
}//prepareForward

void Evaluate::setJoints(const vector<float>& robotJointAngles) {
   kinematics.setJoints(robotJointAngles);
}//setJoints

void Evaluate::updateHumanSkeleton(const vector<Joint>& humanSkeleton) {
   humanJointInfo = humanSkeleton;
   getRotationMatrices();
}//updateHumanSkeleton

void Evaluate::getRotationMatrices() {
   Matrix3f SL, SR, SS, SB; //Shoulder left, Shoulder right, Spine Shoulder, Spine Base
   float angleTorsoX, angleTorsoY, angleTorsoZ;
   double distX, distZ;

   SL.setZero(3, 3);
   SR.setZero(3, 3);
   SS.setZero(3, 3);
   SB.setZero(3, 3);

   SL(0, 0) = humanJointInfo[JointType_ShoulderLeft].Position.X;
   SL(1, 0) = humanJointInfo[JointType_ShoulderLeft].Position.Y;
   SL(2, 0) = humanJointInfo[JointType_ShoulderLeft].Position.Z;

   SR(0, 0) = humanJointInfo[JointType_ShoulderRight].Position.X;
   SR(1, 0) = humanJointInfo[JointType_ShoulderRight].Position.Y;
   SR(2, 0) = humanJointInfo[JointType_ShoulderRight].Position.Z;

   SB(0, 0) = humanJointInfo[JointType_SpineBase].Position.X;
   SB(1, 0) = humanJointInfo[JointType_SpineBase].Position.Y;
   SB(2, 0) = humanJointInfo[JointType_SpineBase].Position.Z;

   SS(0, 0) = humanJointInfo[JointType_SpineShoulder].Position.X;
   SS(1, 0) = humanJointInfo[JointType_SpineShoulder].Position.Y;
   SS(2, 0) = humanJointInfo[JointType_SpineShoulder].Position.Z;
   
   //Z-distance between Shoulder Left and Shoulder Right
   distZ =  humanJointInfo[JointType_ShoulderLeft].Position.Z - 
            humanJointInfo[JointType_ShoulderRight].Position.Z;
   
   //rotation of human body around y-axis
   angleTorsoY = (float)asin(abs(distZ) / (SL - SR).norm());
   if (distZ > 0)
      angleTorsoY *= -1.0;

   ry = Eigen::AngleAxisf(angleTorsoY, Eigen::Vector3f::UnitY());
   
   SB = ry * SB;
   SS = ry * SS;

   //Z-distance between Spine Base and Spine Shoulder
   distZ = SB(2, 0) - SS(2, 0);
   
   //rotation of human body around x-axis
   angleTorsoX = (float)asin(abs(distZ) / (SS - SB).norm());
   if (distZ > 0)
      angleTorsoX *= -1.0;

   rx = Eigen::AngleAxisf(angleTorsoX, Eigen::Vector3f::UnitX());

   //X-distance between Spine Shoulder and Spine Base
   distX = SS(0, 0) - SB(0, 0);

   //rotation of human body around z-axis
   angleTorsoZ = (float)asin(abs(distX) / (SS - SB).norm());
   if (distX > 0)
      angleTorsoZ *= -1.0;

   rz = Eigen::AngleAxisf(angleTorsoZ, Eigen::Vector3f::UnitZ());
}//getRotationMatrices

double Evaluate::evaluate(const int limbID, const bool logError) {
   double error = 0.0, v;
   int humanJoint, robotJoint;
   map<int, int>::iterator it;   
   vector<float> coordinates, coordinatesBase;

   Matrix3f H, V; //Human joint, Nao joint
     
   H.setZero(3, 3);
   V.setZero(3, 3);

   kinematics.prepareForward();
   
   if (logError) {
      if (first) {
         timeStart = clock();
         out14 << 0 << endl;
         first = false;
      }//if
      else          
         out14 << clock() - timeStart << endl;
   }//if

   for (it = humanJointToRobotJoint[limbID].begin(); it != humanJointToRobotJoint[limbID].end(); it++) {
      humanJoint = it->first;
      robotJoint = it->second;

      //Store x, y and z-coordinates of the human joint relative to the base joint of the limb in matrix H.
      H(0, 0) = -(humanJointInfo[humanJoint].Position.X - humanJointInfo[baseHuman[limbID]].Position.X);
      H(1, 0) =   humanJointInfo[humanJoint].Position.Y - humanJointInfo[baseHuman[limbID]].Position.Y;
      H(2, 0) = -(humanJointInfo[humanJoint].Position.Z - humanJointInfo[baseHuman[limbID]].Position.Z);
      
      //Rotations are applied in order to place the coordinates in the same coordinate system as Nao. 
      H = rx * ry * rz * H;
                 
      //Store x, y and z-coordinates of the Nao joint relative to the base joint of the limb in matrix V.
      coordinates = getKinematicsCoordinates(robotJoint);
      coordinatesBase = getKinematicsCoordinates(baseNao[limbID]);
      V(0, 0) = coordinates[0] - coordinatesBase[0]; //x
      V(1, 0) = coordinates[1] - coordinatesBase[1]; //y
      V(2, 0) = coordinates[2] - coordinatesBase[2]; //z

      //The error is calculated by comparing the normalized values of the coordinates stored in matrix H and V.

      /*if (logError) {
         v = ((1 / H.norm()) * H - (1 / V.norm()) * V).norm() * V.norm() / 10.0;
         switch(humanJoint) {
            case JointType_HandLeft:
               out2 << v << endl;
               break;
            case JointType_HandRight:
               out3 << v << endl;
               break;
            case JointType_KneeLeft:
               out4 << v << endl;
               break;
            case JointType_KneeRight:
               out5 << v << endl;
               break;
            case JointType_ElbowLeft:
               out6 << v << endl;
               break;
            case JointType_ElbowRight:
               out7 << v << endl;
               break;
            case JointType_AnkleLeft:
               out8 << v << endl;
               break;
            case JointType_AnkleRight:
               out9 << v << endl;
               break;
         }//switch
      }//if*/     

      error += ((1 / H.norm()) * H - (1 / V.norm()) * V).norm();
   }//for 

   return pow(error * 3.0, 2);
}//setKinematicsAngle

vector<float> Evaluate::getKinematicsCoordinates(const int jointID) {
   NAOKinematics::kmatTable k = kinematics.getForwardEffector((NAOKinematics::Effectors)jointID);

   vector<float> coordinates = vector<float>(3);
   coordinates[0] = (float)k(1, 3);
   coordinates[1] = (float)k(2, 3);
   coordinates[2] = (float)k(0, 3);

   return coordinates;
}//getCoordinates

vector<float> Evaluate::getJointAnglesList() {
   vector<float> angles(ANGLES);
   angles[0] = angles[1] = 0;
   for (int i = 2; i < ANGLES; i++)
      angles[i] = getKinematicsAngle(i);
   return angles;
}//getJointList

double Evaluate::getKinematicsAngle(const int angleID) {
   return kinematics.getJoint(angleID);
}//getKinematicsAngle

void Evaluate::changeKinematicsAngle(const int angleID, const double amount) {
   kinematics.changeJoint(angleID, amount);
}//changeKinematicsAngle

void Evaluate::setKinematicsAngle(const int angleID, const double amount) {   
   kinematics.setJoint(angleID, amount);
}//setKinematicsAngle

double Evaluate::getAngleABC(double* a, double* b, double* c) {
    double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    double bc[3] = {c[0] - b[0], c[1] - b[1], c[2] - b[2]};

    double abVec = sqrt(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
    double bcVec = sqrt(bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2]);

    double abNorm[3] = {ab[0] / abVec, ab[1] / abVec, ab[2] / abVec};
    double bcNorm[3] = {bc[0] / bcVec, bc[1] / bcVec, bc[2] / bcVec};

    double res = abNorm[0] * bcNorm[0] + abNorm[1] * bcNorm[1] + abNorm[2] * bcNorm[2];

    return acos(res) * 180.0 / 3.141592653589793;
}
