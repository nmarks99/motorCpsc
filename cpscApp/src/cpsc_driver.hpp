#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
    public:
        CpscMotorAxis(class CpscMotorController *pC, int axisNo);

        void report(FILE *fp, int level);
        asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
        asynStatus stop(double acceleration);
        asynStatus poll(bool *moving);

    private:
        CpscMotorController *pC_;
        int axisIndex_;
        asynStatus send_vel_and_acc(double base_velocity, double velocity, double acceleration);

    friend class CpscMotorController;
};

class epicsShareClass CpscMotorController : public asynMotorController {
    public:
        CpscMotorController(const char *portName,
                            const char *CpscMotorController,
                            int numAxes, double movingPollPeriod, double idlePollPeriod);
        void report(FILE *fp, int level);
        CpscMotorAxis* getAxis(asynUser *pasynUser);
        CpscMotorAxis* getAxis(int axisNo);

    friend class CpscMotorAxis;
};
