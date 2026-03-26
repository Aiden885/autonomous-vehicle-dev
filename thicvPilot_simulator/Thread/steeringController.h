#ifndef STEERINGCONTROLLER_H
#define STEERINGCONTROLLER_H

#include <iostream>
#include <vector>

#include "controllerPID.h"
#include "ztgeographycoordinatetransform.h"

using namespace std;
class SteeringController
{
private:
    vector<double> pathX;
    vector<double> pathY;
    ControllerPID pidController;

    double steering;

    //..... add more private members you need;

public:
    //..... add more member functions
    // do suggest to use "this->" to indicate your members in your member funtions explicitly.

    SteeringController() : pidController(ControllerPID())
    {

        cout << "SteeringController object is built" << endl;
        // leave out ....
    }
    ~SteeringController() {}
    void setPath() {}
    void calculateSteering()
    {
        startPurePursuit();
        this->steering = 0;
    }

    double getSteering()
    {
        return this->steering;
    }

private:
    // if some functions will be only called by other member functions, put in "private" is better
    void startPurePursuit() { cout << "pure pursuit" << endl; }
};
#endif // STEERINGCONTROLLER_H