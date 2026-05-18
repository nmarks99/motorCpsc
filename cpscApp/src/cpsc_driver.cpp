#include <algorithm>
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
    createParam("CPSC_STEP_SIZE", asynParamInt32, &CpscStepSizeIndex_);
    createParam("CPSC_MOVING_DEADBAND", asynParamFloat64, &CpscMovingDeadbandIndex_);
    createParam("CPSC_FEEDBACK_ENABLE", asynParamInt32, &CpscFeedbackEnableIndex_);
    createParam("CPSC_FEEDBACK_DONE", asynParamInt32, &CpscFeedbackDoneIndex_);

    setDoubleParam(CpscMovingDeadbandIndex_, DEFAULT_MOVING_DEADBAND);
    callParamCallbacks();

    if (numAxes > 3) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but 3 will be used", numAxes);
        numAxes = 3;
    }

    status = pasynOctetSyncIO->connect(CpscMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to CPSC motor controller\n",
                  functionName);
    }
    pasynOctetSyncIO->setInputEos(pasynUserController_, "\r\n", 2);
    pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);

    sprintf(outString_, "/VER");
    writeReadController();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Version: %s\n", inString_);

    sprintf(outString_, "/MODLIST");
    writeReadController();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Modules: %s\n", inString_);

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int CpscMotorCreateController(const char* portName, const char* CpscMotorPortName, int numAxes,
                                         int movingPollPeriod, int idlePollPeriod) {
    new CpscMotorController(portName, CpscMotorPortName, numAxes, movingPollPeriod / 1000.,
                            idlePollPeriod / 1000.);
    return asynSuccess;
}

void CpscMotorController::report(FILE* fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "CPSC Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

CpscMotorAxis* CpscMotorController::getAxis(asynUser* pasynUser) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

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
                fben_cmd += paxis->stage_name_ + " " + std::to_string(freq) + " ";
            }
            double df = 0.0;
            getDoubleParam(CpscDriveFactorIndex_, &df);
            int temp = 0.0;
            getIntegerParam(CpscTemperatureIndex_, &temp);
            sprintf(outString_, "%s%.1lf %d", fben_cmd.c_str(), df, temp);
        } else {
            sprintf(outString_, "FBXT");
            has_moved_ = false;
        }
        status = writeReadController();
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

    int done = 1;
    if (has_moved_) {
        done = closed_loop_ ? fbstatus[FBStatus::DONE] : 1;
    }
    setIntegerParam(CpscFeedbackDoneIndex_, done);

    callParamCallbacks();
    return asyn_status;
}

// =============
// CpscMotorAxis
// =============

CpscMotorAxis::CpscMotorAxis(CpscMotorController* pC, int axisNo, const char* sensor_name)
    : asynMotorAxis(pC, axisNo), pC_(pC), stage_name_(sensor_name) {

    axisIndex_ = axisNo + 1;

    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void CpscMotorAxis::report(FILE* fp, int level) {
    if (level > 0) {
        fprintf(fp, "  Axis #%d, sensor=%s\n", axisNo_, stage_name_.c_str());
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

    asynStatus asyn_status = asynSuccess;

    if (pC_->closed_loop_) {
        pC_->has_moved_ = true;
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
        pC_->writeReadController();
    } else {
        // MOV 1 0 600 100 99 293 CS021-RLS.X 1
        pC_->setIntegerParam(pC_->motorStatusProblem_, 1);
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Open loop moved not implemeted trough motor record\n");
        // constexpr int STEPS_MAX = 50000;
        // // open loop move
        // // MOV [ADDR] [DIR] [FREQ] [RSS] [STEPS] [TEMP] [STAGE] [DF]
        // double current_pos = 0.0;
        // pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &current_pos);
        // printf("Current pos = %.1lf\n", current_pos);
        // printf("Target pos = %.1lf\n", position);
        // double nm_to_move = position - current_pos;
        // int dir = nm_to_move > 0 ? 0 : 1;
        // long steps_to_move = static_cast<long>(fabs(nm_to_move) * STEPS_PER_NANOMETER);
        // steps_to_move = steps_to_move > STEPS_MAX ? STEPS_MAX : steps_to_move;
//
        // // Get frequency, temperature, relative step size, and drive factor
        // int freq = 0;
        // pC_->getIntegerParam(axisNo_, pC_->CpscFrequencyIndex_, &freq);
        // int temp = 0;
        // pC_->getIntegerParam(pC_->CpscTemperatureIndex_, &temp);
        // int rss = 0;
        // pC_->getIntegerParam(pC_->CpscStepSizeIndex_, &rss);
        // double df = 0.0;
        // pC_->getDoubleParam(axisNo_, pC_->CpscDriveFactorIndex_, &df);
//
        // printf("Open loop move: %ld steps (%lf nm) in %d direction\n", steps_to_move, nm_to_move, dir);
        // sprintf(pC_->outString_, "MOV %d %d %d %d %ld %d %s %.1lf", axisIndex_, dir, freq, rss, steps_to_move, temp,
               // stage_name_.c_str(), df);
        // printf("%s\n", pC_->outString_);
        // asyn_status = pC_->writeReadController();
        // if (asyn_status) {
            // asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Move command failed\n");
        // }
    }

    callParamCallbacks();
    return asyn_status;
}

asynStatus CpscMotorAxis::poll(bool* moving) {
    asynStatus asyn_status = asynSuccess;

    // Read position
    sprintf(pC_->outString_, "PGV 4 %d %s", axisIndex_, stage_name_.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "CpscMotorAxis::poll(): Communication error\n");
        return asyn_status;
    }

    // TODO: handle asyn_status
    double position_m = atof((const char*)&pC_->inString_);
    long long_position_nm = DRIVER_RESOLUTION * position_m;
    setDoubleParam(pC_->motorPosition_, long_position_nm); // RRBV [nanometers]
    setDoubleParam(pC_->motorEncoderPosition_, long_position_nm);

    // First check if feedback is active. If so, then no need for delta check
    int done = 1;
    if (pC_->closed_loop_) {
        pC_->getIntegerParam(pC_->CpscFeedbackDoneIndex_, &done);
    } else {
        // Determine if moving with position delta and configurable deadband
        double moving_deadband = 0.0;
        pC_->getDoubleParam(axisNo_, pC_->CpscMovingDeadbandIndex_, &moving_deadband);
        if (!first_poll_) {
            double delta = fabs((double)long_position_nm - last_pos_);
            if (delta > moving_deadband) {
                done = 0;
            }
        }
        last_pos_ = static_cast<double>(long_position_nm);
        first_poll_ = false;
    }

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
