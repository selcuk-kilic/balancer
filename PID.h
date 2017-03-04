#ifndef PID_h
#define PID_h

class PID
{
public:
  float Kp;
  float Ki;
  float Kd;
  float integralComponent;
  float previousError; 
  float lowLimit;
  float highLimit;
  float Ki_Min;
  float Ki_Max;
  bool first_time;

  PID()
  {
    integralComponent = 0;
    previousError = 0; 
    lowLimit = -100;
    highLimit = 100;
    Ki_Min = -100000;
    Ki_Max = 1000000;  
    first_time =true;
  }
  
  float Update(float error, float dt)
  {
    float pidOutput;
    float derror;

    if(first_time) {
       derror = 0;
       first_time = false;
    }
    else {
      derror = error-previousError;
    }
    
    //! Integral component
    integralComponent = integralComponent + Ki*error*dt;
    if (integralComponent > Ki_Max)
    {
      integralComponent = Ki_Max;
    }
    else if (integralComponent < Ki_Min)
    {
      integralComponent = Ki_Min;
    }
    
    //! Calculate PID output
    pidOutput = error * Kp + integralComponent + (derror/dt)*Kd;
    
    //! Check limits
    if(pidOutput > highLimit)
    {
      Ki_Max = integralComponent;
      pidOutput = highLimit;
    }
    else if(pidOutput < lowLimit)
    {
      Ki_Min = integralComponent;
      pidOutput = lowLimit;
    }
    
    previousError = error;
    
    return pidOutput;
  }
  
};


#endif //PID_h
