#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "cpsc_driver.hpp"

// Splits a c style string into a vector<double>
std::vector<int> split_char_arr(const char* msg, char delimiter) {
    std::string s_msg(msg);
    std::replace(s_msg.begin(), s_msg.end(), delimiter, ' ');
    std::istringstream ss(s_msg);
    std::vector<int> v_out{std::istream_iterator<double>(ss), {}};
    return v_out;
}

constexpr long DRIVER_RESOLUTION = 1e9;   // controller reports in meters with nanometer precision
constexpr double STEPS_PER_NANOMETER = 1; // TODO: What is this really?
constexpr double LOW_LIMIT = -4802360.0;  // nanometers (for axis 1 only?)
constexpr double HIGH_LIMIT = 4802361.0;  // nanometers (for axis 1 only?)

// ===================
// CpscMotorController
// ===================

/// \brief Create a new CpscMotorController object
///
/// \param[in] portName             The name of the asyn port that will be created for this driver
/// \param[in] CpscPortName         The name of the drvAsynIPPort that was created previously
/// \param[in] numAxes              The number of axes that this controller supports
/// \param[in] movingPollPeriod     The time between polls when any axis is moving
/// \param[in] idlePollPeriod       The time between polls when no axis is moving
CpscMotorController::CpscMotorController(const char* portName, const char* CpscMotorPortName, int numAxes,
                                         double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    static const char* functionName = "CpscMotorController::CpscMotorController";

    createParam("CPSC_FREQUENCY", asynParamInt32, &CpscFrequencyIndex_);
    createParam("CPSC_TEMPERATURE", asynParamInt32, &CpscTemperatureIndex_);
    createParam("CPSC_DRIVE_FACTOR", asynParamFloat64, &CpscDriveFactorIndex_);
    createParam("CPSC_MOVING_DEADBAND", asynParamFloat64, &CpscMovingDeadbandIndex_);
    createParam("CPSC_FEEDBACK_ENABLE", asynParamInt32, &CpscFeedbackEnableIndex_);
    createParam("CPSC_FEEDBACK_DONE", asynParamInt32, &CpscFeedbackDoneIndex_);

    if (numAxes > 3) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but 3 will be used", numAxes);
        numAxes = 3;
    }

    status = pasynOctetSyncIO->connect(CpscMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to CPSC motor controller\n",
                  functionName);
    }
    sprintf(outString_, "/VER");
    writeReadController();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Version: %s\n", inString_);

    sprintf(outString_, "/MODLIST");
    writeReadController();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Modules: %s\n", inString_);

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/// \breif Creates a new CpscMotorController object.
///
/// Configuration command, called directly or from iocsh
/// \param[in] portName             The name of the asyn port that will be created for this driver
/// \param[in] CpscMotorPortName The name of the drvAsynIPPPort that was created previously
/// \param[in] numAxes              The number of axes that this controller supports
/// \param[in] movingPollPeriod     The time in ms between polls when any axis is moving
/// \param[in] idlePollPeriod       The time in ms between polls when no axis is moving
extern "C" int CpscMotorCreateController(const char* portName, const char* CpscMotorPortName, int numAxes,
                                         int movingPollPeriod, int idlePollPeriod) {
    new CpscMotorController(portName, CpscMotorPortName, numAxes, movingPollPeriod / 1000.,
                            idlePollPeriod / 1000.);
    return asynSuccess;
}

/// \brief Reports on status of the driver
/// \param[in] fp The file pointer on which report information will be written
/// \param[in] level The level of report detail desired
/// If level > 0 then information is printed about each axis.
/// After printing controller-specific information it calls asynMotorController::report()
void CpscMotorController::report(FILE* fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "CPSC Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

/// \brief Returns a pointer to a CpscMotorAxis object
/// \param[in] asynUser structure that encodes the axis index number
/// \returns NULL if the axis number encoded in pasynUser is invalid
CpscMotorAxis* CpscMotorController::getAxis(asynUser* pasynUser) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

/// \brief Returns a pointer to a CpscMotorAxis object
/// \param[in] axisNo Axis index number
/// \returns NULL if the axis number is invalid
CpscMotorAxis* CpscMotorController::getAxis(int axisNo) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus CpscMotorController::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    if (function == CpscFeedbackEnableIndex_) {
        // TODO: build FBEN/FBXT command from axis sensor names and frequencies
        if (value) {
            std::string fben_cmd = "FBEN ";
            for (int i = 0; i < numAxes_; i++) {
                CpscMotorAxis* paxis = getAxis(i);
                if (!paxis) {
                    return asynError;
                }
                int freq = 0;
                getIntegerParam(i, CpscFrequencyIndex_, &freq);
                fben_cmd += paxis->sensor_name_ + " " + std::to_string(freq) + " ";
            }
            sprintf(outString_, "%s%.1lf %d", fben_cmd.c_str(), drive_factor_, temperature_);
        } else {
            sprintf(outString_, "FBXT");
        }
        printf("Enable command: %s\n", outString_);
        // status = writeReadController();
    } else {
        status = asynMotorController::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    return status;
}

asynStatus CpscMotorController::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;
    CpscMotorAxis* paxis;

    paxis = getAxis(pasynUser);
    if (!paxis) {
        return asynError;
    }

    if (function == CpscMovingDeadbandIndex_) {
        printf("Setting moving deadband for axis %d to %lf\n", paxis->axisNo_, value);
        paxis->moving_deadband_ = value;
    } else {
        asyn_status = asynMotorController::writeFloat64(pasynUser, value);
    }

    return asyn_status;
}

// Status indices
enum FBStatus {
    ENABLED = 0,
    DONE = 1,
    INVALID_SP1 = 2,
    INVALID_SP2 = 3,
    INVALID_SP3 = 4,
    POS_ERROR1 = 5,
    POS_ERROR2 = 6,
    POS_ERROR3 = 7
};

asynStatus CpscMotorController::poll() {
    asynStatus asyn_status = asynSuccess;

    // Read feedback status
    sprintf(outString_, "FBST");
    asyn_status = writeReadController();
    // TODO: check asyn_status

    // split input char* by ',' into a std::vector<int>
    std::vector<int> fbstatus = split_char_arr(inString_, ',');
    closed_loop_ = fbstatus[FBStatus::ENABLED];
    setIntegerParam(CpscFeedbackEnableIndex_, closed_loop_);
    int done = fbstatus[FBStatus::DONE];
    setIntegerParam(CpscFeedbackDoneIndex_, done);
    // printf("    Feedback enabled: %d\n", fbstatus[FBStatus::ENABLED]);
    // printf("       Feedback done: %d\n", fbstatus[FBStatus::DONE]);
    // printf("Feedback Invalid SP1: %d\n", fbstatus[FBStatus::INVALID_SP1]);
    // printf("Feedback Invalid SP2: %d\n", fbstatus[FBStatus::INVALID_SP2]);
    // printf("Feedback Invalid SP3: %d\n", fbstatus[FBStatus::INVALID_SP3]);
    // printf("Feedback Pos error 1: %d\n", fbstatus[FBStatus::POS_ERROR1]);
    // printf("Feedback Pos error 2: %d\n", fbstatus[FBStatus::POS_ERROR2]);
    // printf("Feedback Pos error 3: %d\n\n", fbstatus[FBStatus::POS_ERROR3]);
    callParamCallbacks();
    return asyn_status;
}

// =============
// CpscMotorAxis
// =============

CpscMotorAxis::CpscMotorAxis(CpscMotorController* pC, int axisNo, const char* sensor_name)
    : asynMotorAxis(pC, axisNo), pC_(pC), sensor_name_(sensor_name), last_pos_(0.0), first_poll_(true) {

    axisIndex_ = axisNo + 1;

    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void CpscMotorAxis::report(FILE* fp, int level) {
    if (level > 0) {
        fprintf(fp, "  Axis #%d, sensor=%s\n", axisNo_, sensor_name_.c_str());
    }
    asynMotorAxis::report(fp, level);
}

asynStatus CpscMotorAxis::stop(double acceleration) {
    if (pC_->closed_loop_) {
        sprintf(pC_->outString_, "FBES");
    } else {
        sprintf(pC_->outString_, "STP %d", axisIndex_);
    }
    printf("%s\n", pC_->outString_);
    return pC_->writeReadController();
}

asynStatus CpscMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                               double acceleration) {
    if (pC_->closed_loop_) {
        // FBCS [SP1] [ABS] [SP2] [ABS] [SP3] [ABS]
        // [SPx] - setpoint in meters
        // [ABS] - 1 absolute positioning (relative to center of stage)
        //       - 0 relative position (relative to current position)
        position = position / DRIVER_RESOLUTION; // convert to meters before sending
        switch (axisIndex_) {
        case 1:
            sprintf(pC_->outString_, "FBCS %lf 1 0 0 0 0", position);
            break;
        case 2:
            sprintf(pC_->outString_, "FBCS 0 0 %lf 1 0 0", position);
            break;
        case 3:
            sprintf(pC_->outString_, "FBCS 0 0 0 0 %lf 1", position);
            break;
        }
        printf("Closed loop move: %s\n", pC_->outString_);
        // pC_->writeReadController();
    } else {
        // open loop move
        double nm_to_move = position - last_pos_;
        bool dir = nm_to_move > 0 ? 1 : 0;
        int steps_to_move = nm_to_move * STEPS_PER_NANOMETER;
        printf("Open loop move: %d steps (%lf nm) in %d direction\n", steps_to_move, nm_to_move, dir);
    }

    return asynSuccess;
}

asynStatus CpscMotorAxis::poll(bool* moving) {
    asynStatus asyn_status = asynSuccess;

    // Read position
    sprintf(pC_->outString_, "PGV 4 %d %s", axisIndex_, sensor_name_.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "CpscMotorAxis::poll(): Communication error\n");
        return asyn_status;
    }

    // TODO: handle asyn_status
    double position_m = atof((const char*)&pC_->inString_);
    long long_position_nm = DRIVER_RESOLUTION * position_m;
    setDoubleParam(pC_->motorPosition_, long_position_nm); // RRBV [nanometers]

    // Determine if moving with position delta and configurable deadband
    int done = 1;
    if (!first_poll_) {
        double delta = fabs((double)long_position_nm - last_pos_);
        if (delta > moving_deadband_)
            done = 0;
    }
    last_pos_ = static_cast<double>(long_position_nm);
    first_poll_ = false;

    *moving = !done;
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);

    callParamCallbacks();
    return asyn_status;
}

extern "C" int CpscMotorCreateAxis(const char* ctrl_port, int axis_num, const char* sensor_name) {
    CpscMotorController* pC;
    pC = (CpscMotorController*)findAsynPortDriver(ctrl_port);
    if (!pC) {
        printf("CpscMotorCreateAxis: Error: controller %s not found\n", ctrl_port);
        return asynError;
    }
    new CpscMotorAxis(pC, axis_num, sensor_name);
    return asynSuccess;
}

// ==================
// iocsh registration
// ==================

// CpscMotorCreateController
static const iocshArg CpscMotorCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg CpscMotorCreateControllerArg1 = {"drvAsynIPPort name", iocshArgString};
static const iocshArg CpscMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg CpscMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg CpscMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg* const CpscMotorCreateControllerArgs[] = {
    &CpscMotorCreateControllerArg0, &CpscMotorCreateControllerArg1, &CpscMotorCreateControllerArg2,
    &CpscMotorCreateControllerArg3, &CpscMotorCreateControllerArg4};
static const iocshFuncDef CpscMotorCreateControllerDef = {"CpscMotorCreateController", 5,
                                                          CpscMotorCreateControllerArgs};

static void CpscMotorCreateControllerCallFunc(const iocshArgBuf* args) {
    CpscMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

// CpscMotorCreateAxis
static const iocshArg CpscMotorCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg CpscMotorCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg CpscMotorCreateAxisArg2 = {"Sensor name", iocshArgString};
static const iocshArg* const CpscMotorCreateAxisArgs[] = {&CpscMotorCreateAxisArg0, &CpscMotorCreateAxisArg1,
                                                          &CpscMotorCreateAxisArg2};
static const iocshFuncDef CpscMotorCreateAxisDef = {"CpscMotorCreateAxis", 3, CpscMotorCreateAxisArgs};

static void CpscMotorCreateAxisCallFunc(const iocshArgBuf* args) {
    CpscMotorCreateAxis(args[0].sval, args[1].ival, args[2].sval);
}

static void CpscMotorRegister(void) {
    iocshRegister(&CpscMotorCreateControllerDef, CpscMotorCreateControllerCallFunc);
    iocshRegister(&CpscMotorCreateAxisDef, CpscMotorCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(CpscMotorRegister);
}
