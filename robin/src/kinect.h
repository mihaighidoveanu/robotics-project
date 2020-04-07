#ifndef __KINECT_H__
#define __KINECT_H__

typedef int HRESULT;
typedef bool BOOLEAN;

bool SUCCEEDED(int);
bool FAILED(int);

const int JointType_Count = 10;
const int JointType_AnkleLeft = 10;
const int JointType_AnkleRight = 10;
const int JointType_HipLeft = 10;
const int JointType_HipRight = 10;
const int JointType_ElbowLeft = 10;
const int JointType_ElbowRight = 10;
const int JointType_HandLeft = 10;
const int JointType_HandRight = 10;
const int JointType_SpineBase = 10;
const int JointType_SpineShoulder = 10;
const int JointType_ShoulderLeft = 10;
const int JointType_ShoulderRight = 10;
const int JointType_KneeLeft = 10;
const int JointType_KneeRight = 10;

const int BODY_COUNT = 10;


class Point{
    public:
        double X, Y, Z;
};

class Joint {
    public:
        Point Position;
};

class IBody{
    public:
        int GetJoints(int, Joint[JointType_Count]);
        int get_IsTracked(BOOLEAN*);
        int Release();

};

class IBodyFrame {
    public:
        int GetAndRefreshBodyData(const int&, IBody* [10] );
        int Release();
    
};

class IBodyFrameReader{ 
    public :
        int AcquireLatestFrame(IBodyFrame**);
        int Release();
};

class IBodyFrameSource{ 
    public:
        int OpenReader(IBodyFrameReader**);
        int Release();
};

class KinectSensor{

    public:

        int Open();
        int get_BodyFrameSource(IBodyFrameSource**);
        int get_IsAvailable(BOOLEAN*);
        int Release();
     

};

int GetDefaultKinectSensor(KinectSensor**);
#endif
