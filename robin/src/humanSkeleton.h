#ifndef __HUMANSKELETON_H__
#define __HUMANSKELETON_H__

#include <iostream>
#include <vector>
#include <kinect.h>
using namespace std;

template<typename T>
void release(T& pointer) {
	if (pointer) {
		pointer->Release();
		pointer = nullptr;
	}//if
}//release

class HumanSkeleton {
	public:
		~HumanSkeleton();
		HumanSkeleton();
		bool getSkeletonCoordinates(vector<Joint>& jointCoordinates);
        KinectSensor * sensor;
	private:
		/* IKinectSensor* sensor; */
		IBodyFrameReader* bodyFrameReader;      
      Joint joints[JointType_Count];
		void getSkeletonData(const int bodyCount, IBody** bodies);
};

#endif
