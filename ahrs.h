
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

typedef void (*Reporter)(float yaw, float pitch, float roll);

class AHRS
{
public:
  void setup(Reporter sendResult);

private:
  static void filterUpdateIsr();
  static float convertRawAcceleration(int aRaw);
  static float convertRawGyro(int gRaw);

};

