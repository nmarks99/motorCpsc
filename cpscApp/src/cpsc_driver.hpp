#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
    public:
        CpscMotorAxis(class CpscMotorController *pC, int axisNo);

        void report(FILE *fp, int level);
        asynStatus stop(double acceleration);
        asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
        asynStatus poll(bool *moving);
        asynStatus setClosedLoop(bool closedLoop);
        // asynStatus poll(bool *moving);

    private:
        CpscMotorController *pC_;
        int axisIndex_;
        bool once;
        double mres;

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
