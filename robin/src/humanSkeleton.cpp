#include "humanSkeleton.h"

bool HumanSkeleton::getSkeletonCoordinates(vector<Joint>& jointCoordinates) {
    std::vector<double> data = this->reader.getdata();
    for (int i = 0; i < data.size(); i = i + 4) {
        Point point(data[i], data[i] + 1, data[i] + 2);
        double proba = data[i + 3]; 
        // ignore probability
        Joint joint(point);
        int idx = i / 4;
        this->joints[idx] = joint;
        jointCoordinates[idx] = joint;
    }
}

/* int main(){ */
/*     vector<Joint> coordinates(JointType_Count); */
/*     HumanSkeleton h; */
/*     h.getSkeletonCoordinates(coordinates); */ 
/*     for (auto joint : coordinates) { */
/*         std::cerr << __LINE__ << ": " << "#joint.Position.X" << " = " << joint.Position.X << std::endl; */
/*     } */
/*     return 0; */
/* } */
