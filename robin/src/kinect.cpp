#include "kinect.h"

bool SUCCEEDED(int x){
    return true;
}

bool FAILED(int x){
    return true;
}

int IBody::GetJoints(int x, Joint y[JointType_Count] ){
    return 1;
}

int IBody::get_IsTracked(BOOLEAN* x){
    return 1;
}

int IBody::Release(){
    return 1;
}

int IBodyFrame::GetAndRefreshBodyData(const int &x, IBody* y[10] ){
    return 1;
}

int IBodyFrame::Release(){
    return 1;
}

int IBodyFrameReader::AcquireLatestFrame(IBodyFrame** x){
    return 1;
}

int IBodyFrameReader::Release(){
    return 1;
}

int IBodyFrameSource::OpenReader(IBodyFrameReader** x){
    return 1;
}

int IBodyFrameSource::Release(){
    return 1;
}

int KinectSensor::Open(){
    return 1;
}

int KinectSensor::get_BodyFrameSource(IBodyFrameSource** x){
    return 1;
}

int KinectSensor::get_IsAvailable(BOOLEAN* x){
    return 1;
}

int KinectSensor::Release(){
    return 1;
}

int GetDefaultKinectSensor(KinectSensor** x){
    return 1;
}
