//
// Created by doron.pinsky on 29/01/2020.
//
#include <iostream>
#include <cmath>

#define _USE_MATH_DEFINES
#ifndef ASTARDUBINS_ROBOT_H
#define ASTARDUBINS_ROBOT_H


class Robot {
    double uMax;
    double vMax;
    double vMin;
    double eta;
    int numOfOreintations;

public:
    Robot();
    void setuMax(double uMaxIn);
    void setvMax(double vMaxIn);
    void setvMin(double vMinIn);
    void setEta(double etaIn);
    void setNumOfOreintations(int orientationIn);
    double getuMax();
    double getvMax();
    double getvMin();
    double getEta();
    int getNumOfOrientation();
    void printInfo();
};

// define a constructor
Robot::Robot()
{
    uMax = 0.5;
    vMin = 0.5;
    vMax = 1;
    eta = M_PI / 2;
    numOfOreintations = 8;
}

void Robot::setuMax(double uMaxIn)
{
    uMax = uMaxIn;
}

void Robot::setvMax(double vMaxIn)
{
    vMax = vMaxIn;
}

void Robot::setvMin(double vMinIn)
{
    vMin = vMinIn;
}

void Robot::setEta(double etaIn)
{
    eta = etaIn;
}

void Robot::setNumOfOreintations(int orientationIn)
{
    numOfOreintations = orientationIn;
}

double Robot::getuMax()
{
    return uMax;
}

double Robot::getvMax()
{
    return vMax;
}

double Robot::getvMin()
{
    return vMin;
}

double Robot::getEta()
{
    return eta;
}

int Robot::getNumOfOrientation()
{
    return numOfOreintations;
}

void Robot::printInfo()
{
    std::cout<<"uMax = "<<uMax<<" [r/s]\n"<<"vMax = "<<vMax<<" [m/s]\n"<<"vMin = "<<vMin<<" [m/s]\n";
    std::cout<<"eta = "<<eta<<"\n"<<"number of orientations = "<<numOfOreintations<<"\n";
}



#endif //ASTARDUBINS_ROBOT_H
