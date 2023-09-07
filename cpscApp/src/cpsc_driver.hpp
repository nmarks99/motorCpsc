#include "asynMotorController.h"
#include "asynMotorAxis.h"

class epicsShareClass CpscMotorAxis : public asynMotorAxis {
    public:
        CpscMotorAxis(class CpscMotorController *pC, int axisNo);

        void report(FILE *fp, int level);

    private:
        CpscMotorController *pC_;
        int axisIndex_;

    friend class CpscMotorController;
};

class epicsShareClass CpscMotorController : public asynMotorController {
    public:
        CpscMotorController(const char *portName, const char *CpscMotorController, int numAxes, double movingPollPeriod, double idlePollPeriod);

        void report(FILE *fp, int level);

        void dummy_function(void);

        CpscMotorAxis* getAxis(asynUser *pasynUser);

        CpscMotorAxis* getAxis(int axisNo);

    friend class CpscMotorAxis;
};
