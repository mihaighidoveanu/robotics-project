#ifndef __KINECT_H__
#define __KINECT_H__

#include <vector>

typedef int HRESULT;
typedef bool BOOLEAN;

bool SUCCEEDED(int);
bool FAILED(int);

const int JointType_Count = 19;
const int JointType_AnkleLeft = 8;
const int JointType_AnkleRight = 14;
const int JointType_HipLeft = 6;
const int JointType_HipRight = 12;
const int JointType_ElbowLeft = 4;
const int JointType_ElbowRight = 10;
const int JointType_HandLeft = 5;
const int JointType_HandRight = 11;
const int JointType_SpineBase = 2; // missing info, might replace with mean of hips
const int JointType_SpineShoulder = 0; // neck
const int JointType_ShoulderLeft = 3;
const int JointType_ShoulderRight = 9;
const int JointType_KneeLeft = 7;
const int JointType_KneeRight = 13;
// not used here
const int JointType_Nose = 13;
const int JointType_EyeLeft = 13;
const int JointType_EyeRight = 13;
const int JointType_EarLeft = 13;
const int JointType_EarRight = 13;

const int BODY_COUNT = 1;


class Point{
    public:
        Point() = default;
        Point(double X, double Y, double Z);
        double X, Y, Z;
};

class Joint {
    public:
        Joint() = default;
        Joint(const Point&);
        Point Position;
};

class Reader {
    public:
        Reader() = default;
        std::vector<double> getdata();
    private:
        std::vector<double> data;
};

#endif
