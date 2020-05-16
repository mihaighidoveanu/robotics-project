#ifndef __HUMANSKELETON_H__
#define __HUMANSKELETON_H__

#include <iostream>
#include <vector>
#include "kinect.h"

using namespace std;

class HumanSkeleton {
	public:
		~HumanSkeleton() = default;
		HumanSkeleton() = default;
		bool getSkeletonCoordinates(vector<Joint>& jointCoordinates);
	private:
      Joint joints[JointType_Count];
      Reader reader;
};

#endif
