#ifndef CONTROLLERPID_H
#define CONTROLLERPID_H

#include <iostream>
using namespace std;

class ControllerPID
{

private:
    double pError, iError, dError;
    double kP, kI, kD;

public:
    ControllerPID()
    {
        cout << "ControllerPID object is built" << endl;
    }
    ~ControllerPID() {}
    void update() {}
};

#endif // CONTROLLERPID_H