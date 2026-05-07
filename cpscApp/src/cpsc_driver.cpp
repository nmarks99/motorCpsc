#include <algorithm>
#include <array>
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

constexpr long MULT = 1000000000;        // controller reports in meters with nanometer precision
constexpr double LOW_LIMIT = -4802360.0; // nanometers (for axis 1 only?)
constexpr double HIGH_LIMIT = 4802361.0; // nanometers (for axis 1 only?)

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
    : asynMotorAxis(pC, axisNo), pC_(pC), sensor_name_(sensor_name) {

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
    return pC_->writeReadController();
}

asynStatus CpscMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                               double acceleration) {
    if (pC_->closed_loop_) {
        // FBCS [SP1] [ABS] [SP2] [ABS] [SP3] [ABS]
        // [SPx] - setpoint in meters
        // [ABS] - 1 absolute positioning (relative to center of stage)
        //       - 0 relative position (relative to current position)
        position = position / MULT; // convert to meters before sending
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
        printf("Move command: %s\n", pC_->outString_);
        // pC_->writeReadController();
    } else {
        printf("Open loop move not implemeted\n");
        // open loop move
    }

    return asynSuccess;
}

asynStatus CpscMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "CpscMotorAxis::home not implemented\n");
    // TODO: reimplement with controller-level fben_ and sensor_name_
    // std::map<int, const char*> axis_map = {
    //     {1, "FBCS 0.0 1 0.0 0 0.0 0"},
    //     {2, "FBCS 0.0 0 0.0 1 0.0 0"},
    //     {3, "FBCS 0.0 0 0.0 0 0.0 1"},
    // };
    // if (fben) {
    //     sprintf(pC_->outString_, "%s", axis_map[axisIndex_]);
    // }
    // asynStatus status = pC_->writeReadController();
    // return status;
    return asynSuccess;
}

asynStatus CpscMotorAxis::poll(bool* moving) {
    asynStatus asyn_status = asynSuccess;

    // Read position
    sprintf(pC_->outString_, "PGV 4 %d %s", axisIndex_, sensor_name_.c_str());
    asyn_status = pC_->writeReadController();

    // TODO: handle asyn_status
    double position_m = atof((const char*)&pC_->inString_);
    long long_position_nm = MULT * position_m;
    setDoubleParam(pC_->motorPosition_, long_position_nm); // RRBV [nanometers]

    // TODO: implement moving check
    int done = 1;
    *moving = !done;
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);
    // int is_moving = 0;
    // pC_->getIntegerParam(axisNo_, pC_->motorStatusMoving_, &is_moving);
    // *moving = is_moving;
    callParamCallbacks();
    return asyn_status;
}

asynStatus CpscMotorAxis::setClosedLoop(bool closedLoop) {
    // TODO: reimplement as per-axis open-loop/closed-loop toggle
    // Previously this triggered controller-wide FBEN/FBXT, which is now
    // handled by the CPSC_FEEDBACK_ENABLE parameter in writeInt32().
    //
    // if (closedLoop) {
    //     if (not pC_->fben_) {
    //         // build FBEN command from per-axis sensor names and frequencies
    //     }
    // } else {
    //     sprintf(pC_->outString_, "FBXT");
    //     pC_->writeReadController();
    // }
    printf("TODO: setClosedLoop(%s) on axis %d\n", closedLoop ? "true" : "false", axisNo_);
    return asynSuccess;
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
