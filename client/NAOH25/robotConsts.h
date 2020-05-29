/**
 * robot_consts.h
 *
 *  Created on: Dec 9, 2010
 *      Author: trs
 */

#ifndef ROBOT_CONSTS_H_
#define ROBOT_CONSTS_H_
#include <vector>
#include <map>
#include <string>

namespace KDeviceLists
{
   const int ELBOW_SIZE = 2;
   const int SHOULDER_SIZE = 0;
   const int ANKLE_SIZE = 4;
   const int KNEE_SIZE = 3;
   const int HIP_SIZE = 0;
   const int NECK_SIZE = 0;

	enum ChainHeadNames
	{
	    YAW = 0, PITCH, HEAD_SIZE
	};

	enum ChainArmNames
	{
	    SHOULDER_PITCH = 0, SHOULDER_ROLL, ELBOW_YAW, ELBOW_ROLL, WRIST_YAW, /*HAND,*/ARM_SIZE
	};

	enum ChainLegNames
	{
	    HIP_YAW_PITCH = 0, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL, LEG_SIZE
	};

	enum ChainsNames
	{
		CHAIN_HEAD = 0, CHAIN_L_ARM, CHAIN_L_ELBOW, CHAIN_L_SHOULDER, CHAIN_R_ARM, CHAIN_R_ELBOW, CHAIN_R_SHOULDER, CHAIN_L_LEG, CHAIN_L_ANKLE,
      CHAIN_L_KNEE, CHAIN_L_HIP, CHAIN_R_LEG, CHAIN_R_ANKLE, CHAIN_R_KNEE, CHAIN_R_HIP, CHAINS_SIZE
	};

	enum JointNames
	{
	    HEAD = 0, L_ARM = HEAD_SIZE, L_LEG = L_ARM + ARM_SIZE, R_LEG = L_LEG + LEG_SIZE, R_ARM = R_LEG + LEG_SIZE, NUMOFJOINTS = R_ARM + ARM_SIZE

	};

   enum Rotations 
   {
      HeadYaw = 0, HeadPitch, 
      LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, 
      LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll,
      RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll,
      RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw
   };

   enum Limbs
   {
      leftArm = 0, leftLeg, rightLeg, rightArm
   };
};
#endif /* ROBOT_CONSTS_H_ */
