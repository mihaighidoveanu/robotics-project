#include "humanSkeleton.h"

HumanSkeleton::HumanSkeleton() {	
	sensor = nullptr; 
	bodyFrameReader = nullptr;
}//constructor
	
HumanSkeleton::~HumanSkeleton() {	
	release(sensor);
	release(bodyFrameReader);
}//destructor

bool HumanSkeleton::getSkeletonCoordinates(vector<Joint>& jointCoordinates) {
	HRESULT hr = GetDefaultKinectSensor(&sensor);
	if (SUCCEEDED(hr)) {
		hr = sensor->Open();
		if (SUCCEEDED(hr)) {
			IBodyFrameSource* bodyFrameSource = nullptr;
			hr = sensor->get_BodyFrameSource(&bodyFrameSource);

			if (SUCCEEDED(hr))
				hr = bodyFrameSource->OpenReader(&bodyFrameReader);
			release(bodyFrameSource);
		}//if
	}//if
	if (sensor == nullptr || FAILED(hr)) {
		cerr << "Can't get default kinect sensor." << endl;
		exit(3);
	}//if
	
	IBodyFrame* bodyFrame = nullptr;
	hr = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
	if (SUCCEEDED(hr)) {
		IBody* bodies[BODY_COUNT] = {nullptr};
		hr = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);
		if (SUCCEEDED(hr)) {
			getSkeletonData(BODY_COUNT, bodies);
			for (int i = 0; i < BODY_COUNT; i++)
				release(bodies[i]);
			release(bodyFrame);
		}//if
      for (int i = 0; i < JointType_Count; i++)
         jointCoordinates[i] = joints[i];
      return true;
	}//if
	else if (sensor) {
		BOOLEAN sensorAvailable = false;
		hr = sensor->get_IsAvailable(&sensorAvailable);
      if (SUCCEEDED(hr) && !sensorAvailable)
			cerr << "No available sensor is found." << endl;
	}//else if
	else
		cerr << "Trouble reading the body frame." << endl;		
	return false;
}//getSkeletonCoordinates

void HumanSkeleton::getSkeletonData(const int bodyCount, IBody** bodies) {
	for (int i = 0; i < bodyCount; i++) {
		IBody* body = bodies[i];
		BOOLEAN isTracked = false;
		HRESULT hr = body->get_IsTracked(&isTracked);
		if (FAILED(hr) || !isTracked)
			continue;		
		hr = body->GetJoints(JointType_Count, joints);
	}//for
}//processBodies
