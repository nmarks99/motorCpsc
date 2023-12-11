#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

static constexpr char CpscFrequencyXString[] = "CPSC_FREQUENCY_X";
static constexpr char CpscFrequencyYString[] = "CPSC_FREQUENCY_Y";
static constexpr char CpscFrequencyZString[] = "CPSC_FREQUENCY_Z";
static constexpr char CpscTemperatureString[] = "CPSC_TEMPERATURE";

static constexpr int DEFAULT_FREQUENCY = 600;
static constexpr int DEFAULT_TEMPERATURE = 293;


class epicsShareClass CpscMotorAxis : public asynMotorAxis {
    public:
        CpscMotorAxis(class CpscMotorController *pC, int axisNo);

        void report(FILE *fp, int level);
        asynStatus stop(double acceleration);
        asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
        asynStatus poll(bool *moving);
        asynStatus setClosedLoop(bool closedLoop);
        asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
        
    private:
        CpscMotorController *pC_;
        int axisIndex_;
        bool once;
        double mres;
        bool fben; // feedback enabled
    
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

    protected:
        int CpscFrequencyX_;
        int CpscFrequencyY_;
        int CpscFrequencyZ_;
        int CpscTemperature_;
    
        int frequencyX = DEFAULT_FREQUENCY;
        int frequencyY = DEFAULT_FREQUENCY;
        int frequencyZ = DEFAULT_FREQUENCY;
        int temperature = DEFAULT_TEMPERATURE;

    friend class CpscMotorAxis;

};
