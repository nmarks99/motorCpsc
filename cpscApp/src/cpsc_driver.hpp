#include <string>

#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

static constexpr double DEFAULT_MOVING_DEADBAND = 50.0; // nm
// static constexpr double DEFAULT_DRIVE_FACTOR = 1.0;
// static constexpr int DEFAULT_FREQUENCY = 600.0; // Hz
// static constexpr int DEFAULT_TEMPERATURE = 293;

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
  public:
    CpscMotorAxis(class CpscMotorController* pC, int axisNo, const char* sensorName);

    void report(FILE* fp, int level) override;
    asynStatus stop(double acceleration) override;
    asynStatus move(double position, int relative, double min_velocity, double max_velocity,
                    double acceleration) override;
    asynStatus poll(bool* moving) override;

  private:
    CpscMotorController* pC_;
    int axisIndex_;
    std::string stage_name_;
    double last_pos_ = 0.0;
    bool first_poll_ = true;

    friend class CpscMotorController;
};

class epicsShareClass CpscMotorController : public asynMotorController {
  public:
    CpscMotorController(const char* portName, const char* CpscMotorPortName, int numAxes,
                        double movingPollPeriod, double idlePollPeriod);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
    asynStatus poll() override;
    void report(FILE* fp, int level) override;
    CpscMotorAxis* getAxis(asynUser* pasynUser) override;
    CpscMotorAxis* getAxis(int axisNo) override;

  private:
    bool closed_loop_ = false;
    bool has_moved_ = false;

  protected:
    static constexpr int NUM_PARAMS = 7;
    int CpscFrequencyIndex_;
    int CpscTemperatureIndex_;
    int CpscDriveFactorIndex_;
    int CpscStepSizeIndex_;
    int CpscMovingDeadbandIndex_;
    int CpscFeedbackEnableIndex_;
    int CpscFeedbackDoneIndex_;

    friend class CpscMotorAxis;
};
