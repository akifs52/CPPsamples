#include <iostream>

class PID
{
public:
    double Kp,Ki,Kd;
    double prevError, integral;

    PID(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd), prevError(0), integral(0) {}

    double compute(double setpoint, double measured, double dt){

        double error = setpoint-measured;
        integral = error*dt;
        double derivative = (error-prevError) / dt;
        prevError =error;

        return (Kp*error)+ (Ki*integral) + (Kd*derivative);
    }
};
using namespace std;

int main()
{
    PID pitchControl(1.5,0.5,0.2);
    double targetPitch = 5.0;
    double currentPitch = 0.0;
    double dt= 0.1;

    for(int i=0; i<100; i++)
    {
        double controlSignal = pitchControl.compute(targetPitch,currentPitch,dt);
        currentPitch += controlSignal * dt;

        std::cout << "Step " << i << ": Control Signal = " << controlSignal
                  << ", Current Pitch = " << currentPitch << std::endl;
    }
    return 0;
}
