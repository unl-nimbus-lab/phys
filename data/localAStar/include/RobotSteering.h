#ifndef ROBOTSTEERING_H
#define ROBOTSTEERING_H



class RobotSteering{
public:
    float maxSpeed;
    float maxAng2forward;//maksymalny kat, przy którym robot będzie jechał już do przodu
    float acceleration;
    float posTolerance;
    float rotConst;

    float calculateAngularSpeed(float dist, float rot_dist);
    float calculateLinearSpeed(float dist, float rot_dist);
    RobotSteering();

};

#endif
