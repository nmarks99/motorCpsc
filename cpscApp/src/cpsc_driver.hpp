#include <string>

#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

static constexpr int DEFAULT_FREQUENCY = 600;
static constexpr int DEFAULT_TEMPERATURE = 293;
static constexpr double DEFAULT_DRIVE_FACTOR = 1.0;

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
  public:
    CpscMotorAxis(class CpscMotorController* pC, int axisNo, const char* sensorName);

    void report(FILE* fp, int level);
    asynStatus stop(double acceleration);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity,
                    double acceleration);
    asynStatus poll(bool* moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);

  private:
    CpscMotorController* pC_;
    int axisIndex_;
    std::string sensor_name_;

    friend class CpscMotorController;
};

class epicsShareClass CpscMotorController : public asynMotorController {
  public:
    CpscMotorController(const char* portName, const char* CpscMotorPortName, int numAxes,
                        double movingPollPeriod, double idlePollPeriod);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    asynStatus poll();
    void report(FILE* fp, int level);
    CpscMotorAxis* getAxis(asynUser* pasynUser);
    CpscMotorAxis* getAxis(int axisNo);

  private:
    bool closed_loop_ = false;
    int temperature_ = DEFAULT_TEMPERATURE;
    double drive_factor_ = DEFAULT_DRIVE_FACTOR;

  protected:
    static constexpr int NUM_PARAMS = 5;
    int CpscFrequencyIndex_;
    int CpscTemperatureIndex_;
    int CpscDriveFactorIndex_;
    int CpscFeedbackEnableIndex_;
    int CpscFeedbackDoneIndex_;

    friend class CpscMotorAxis;
};
