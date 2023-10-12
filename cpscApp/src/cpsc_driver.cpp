#include <iomanip>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

#include "cpsc_driver.hpp"
#include "utils.hpp"
using utils::Color;
using utils::stylize_string;

constexpr int PREC = 9; // controller reports in meters with nanometer precision
constexpr double LOW_LIMIT = -0.004802360; // meters (for axis 1 only?)
constexpr double HIGH_LIMIT = 0.004802361; // meters (for axis 1 only?)
const long MULT = static_cast<long>(1 * pow(10, PREC));


// ===================
// CpscMotorController
// ===================

constexpr int NUM_PARAMS = 0;

/// \brief Create a new CpscMotorController object
///
/// \param[in] portName             The name of the asyn port that will be created for this driver
/// \param[in] CpscPortName The name of the drvAsynIPPort that was created previously 
/// \param[in] numAxes              The number of axes that this controller supports 
/// \param[in] movingPollPeriod     The time between polls when any axis is moving 
/// \param[in] idlePollPeriod       The time between polls when no axis is moving 
CpscMotorController::CpscMotorController(const char *portName, const char *CpscMotorPortName, int numAxes,
                                         double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                        0, // No additional interfaces beyond the base class
                        0, // No additional callback interfaces beyond those in base class
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                        1, // autoconnect
                        0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    CpscMotorAxis *pAxis;
    static const char *functionName = "CpscMotorController::CpscMotorController";
    
    // only feedback for 3 axes
    if (numAxes > 3) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but 3 will be used", numAxes);
        numAxes = 3;
    }

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(CpscMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s: cannot connect to CPSC motor controller\n",
        functionName);
    }
    
    // Create CpscMotorAxis object for each axis
    // if not done here, user must call CpscMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new CpscMotorAxis(this, axis);
    }

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
extern "C" int CpscMotorCreateController(const char *portName, const char *CpscMotorPortName,
                                         int numAxes, int movingPollPeriod, int idlePollPeriod)
{
    CpscMotorController *pCpscMotorController = new CpscMotorController(portName, CpscMotorPortName, numAxes,
                                                                        movingPollPeriod/1000., idlePollPeriod/1000.);
    pCpscMotorController = NULL;
    return(asynSuccess);
}

/// \brief Reports on status of the driver
/// \param[in] fp The file pointer on which report information will be written
/// \param[in] level The level of report detail desired
/// If level > 0 then information is printed about each axis.
/// After printing controller-specific information it calls asynMotorController::report()
void CpscMotorController::report(FILE *fp, int level) {
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
CpscMotorAxis* CpscMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

/// \brief Returns a pointer to a CpscMotorAxis object
/// \param[in] axisNo Axis index number
/// \returns NULL if the axis number is invalid
CpscMotorAxis* CpscMotorController::getAxis(int axisNo) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(axisNo));
}


// =============
// CpscMotorAxis
// =============

/// \breif Creates a new VirtualMotorAxis object.
/// \param[in] pC Pointer to the VirtualMotorController to which this axis belongs. 
/// \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
/// 
/// Initializes register numbers, etc.
/// Note: the following constructor needs to be modified to accept the stepSize argument if CpscMotorCreateAxis
/// will be called from iocsh, which is necessary for controllers that work in EGU (engineering units) instead of steps.
CpscMotorAxis::CpscMotorAxis(CpscMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC)
{

    axisIndex_ = axisNo + 1;
    this->once = true;
    this->fben = false; // will be updated from poll method

    // enables setClosedLoop function:
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);

    // get MRES and set limits (not working?)
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &this->mres);
    setDoubleParam(pC_->motorLowLimit_, LOW_LIMIT/mres);
    setDoubleParam(pC_->motorHighLimit_, HIGH_LIMIT/mres);

    asynPrint(
        pasynUser_,
        ASYN_REASON_SIGNAL,
        stylize_string("CpscMotorAxis created with axis index %d\n", Color::GREEN).c_str(),
        axisIndex_
    );

    callParamCallbacks();
    
}

/// \brief Report on the axis
void CpscMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}


/// \brief Stop the axis
asynStatus CpscMotorAxis::stop(double acceleration) {
    asynStatus status;
    if (fben) {
        // Feedback mode E-stop
        asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBES\n");
        sprintf(pC_->outString_, "FBES"); 
    }
    else {
        // Open-loop mode stop axis
        asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "STP %d\n", axisIndex_);
        sprintf(pC_->outString_, "STP %d", axisIndex_);
    }
    status = pC_->writeReadController();
    return status;
}


/// \brief Move the axis
asynStatus CpscMotorAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration) {
    // Go to setpoint
    // FBCS [SP1] [ABS] [SP2] [ABS] [SP3] [ABS]
    // [SPx] - setpoint in meters
    // [ABS] - 1 absolute positioning (relative to center of stage)
    //       - 0 relative position (relative to current position)
    asynStatus status;
    
    // Only allowing closed-loop motion from EPICS for now.
    // Use MOV commands directly if open-loop motion is needed.
    if (fben) {
        position = position / MULT; // convert to meters before sending

        // sets the setpoint for the current axis to "position" absolute
        // other axes are set to move 0.0 relative to their current position
        switch (axisIndex_) {
            case 1:
                asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS %.9lf 1 0 0 0 0\n", position);
                // sprintf(pC_->outString_, "FBCS %lf 1 0 0 0 0", position);
                break;
            case 2: 
                asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS 0 0 %.9lf 1 0 0\n", position);
                // sprintf(pC_->outString_, "FBCS 0 0 %lf 1 0 0", position);
                break;
            case 3: 
                asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS 0 0 0 0 %.9lf 1\n", position);
                // sprintf(pC_->outString_, "FBCS 0 0 0 0 %lf 1", position);
                break;
            default:
                asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Invalid axis index %d\n", axisIndex_);
        }
    }
    else {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  stylize_string("Warning(Axis %d): Move aborted. Feedback mode is disabled\n", Color::YELLOW).c_str(),
                  axisIndex_
        );
    }
    // status = pC_->writeReadController();
    return status;
}

/// \brief Poll the axis
asynStatus CpscMotorAxis::poll(bool *moving) {
    asynStatus asyn_status;
    double position_m = 0.0;
    long long_position_nm = 0;
    int done = 1;
    std::vector<double> status;

    // Read position
    sprintf(pC_->outString_, "PGV 4 %d CBS10-RLS", axisIndex_);
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  stylize_string("Error(Axis %d): read position failed\n", Color::RED).c_str(), axisIndex_);
        setIntegerParam(pC_->motorStatusProblem_, asyn_status);
        callParamCallbacks();
        return asyn_status ? asynError : asynSuccess;
    }

    // Adjust MRES to obtain the desired units of the readback values
    // MRES = 1.0 -> nanometers
    // MRES = 1e-3 -> micrometers
    // MRES = 1e-6 -> millimeters
    // MRES = 1e-9 -> meters
    position_m = atof((const char *) &pC_->inString_);
    long_position_nm = MULT * position_m;
    setDoubleParam(pC_->motorPosition_, long_position_nm); // RRBV [nanometers]
    
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
    // Read status 
    sprintf(pC_->outString_, "FBST");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                  stylize_string("Error(Axis %d): read status failed\n", Color::RED).c_str(), axisIndex_);
        setIntegerParam(pC_->motorStatusProblem_, asyn_status);
        callParamCallbacks();
        return asyn_status ? asynError : asynSuccess;
    }

    // split input char* by ',' into a std::vector<double>
    status = utils::split_char_arr(pC_->inString_, ',');

    // Handle error codes
    if (not status.at(ENABLED)) {
        if (once) {
            asynPrint(
                pasynUser_,
                ASYN_TRACE_ERROR,
                stylize_string("Warning(Axis %d): Feedback mode is disabled\n", Color::YELLOW).c_str(),
                axisIndex_
            );
            once = false;
        }
        fben = false;
        done = 1;
        setIntegerParam(pC_->motorStatusDone_, done);
        setIntegerParam(pC_->motorStatusMoving_, !done);
        *moving = 0;
    }
    else {
        fben = true;
        done = status.at(FBStatus::DONE);
        setIntegerParam(pC_->motorStatusDone_, done);
        setIntegerParam(pC_->motorStatusMoving_, !done);
        *moving = !status.at(1);

        if (status.at(FBStatus::INVALID_SP1)) {
            asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                      stylize_string("Error: Invalid setpoint on axis 1\n", Color::RED).c_str(),axisIndex_);
        }
        if (status.at(FBStatus::INVALID_SP2)) {
            asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                      stylize_string("Error: Invalid setpoint on axis 2\n", Color::RED).c_str(),axisIndex_);
        }
        if (status.at(FBStatus::INVALID_SP3)) {
            asynPrint(pasynUser_,ASYN_TRACE_ERROR,
                      stylize_string("Error: Invalid setpoint on axis 3\n", Color::RED).c_str(),axisIndex_);
        }
    }
    
    // skip:
    setIntegerParam(pC_->motorStatusProblem_, asyn_status ? 1 : 0);
    callParamCallbacks();
    return asyn_status ? asynError : asynSuccess;
}


/// \brief Enable closed loop (Servodrive mode)
asynStatus CpscMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus status;

    if (closedLoop) {
        // enable closed loop
        asynPrint(pasynUser_,ASYN_TRACE_ERROR, "(Axis %d): Feedback mode enabled\n",axisIndex_);
        sprintf(pC_->outString_, "FBEN CBS10-RLS 300 CBS10-RLS 300 CBS10-RLS 300 1 293");
    }
    else {
        // disable closed loop
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "(Axis %d): Feedback mode disabled\n", axisIndex_);
        sprintf(pC_->outString_, "FBXT");
    }

    status = pC_->writeReadController();
    return status;
}



// ==================
// iosch registration
// ==================

static const iocshArg CpscMotorCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg CpscMotorCreateControllerArg1 = {"VMC port name", iocshArgString};
static const iocshArg CpscMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg CpscMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg CpscMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const CpscMotorCreateControllerArgs[] = {&CpscMotorCreateControllerArg0,
                                                             &CpscMotorCreateControllerArg1,
                                                             &CpscMotorCreateControllerArg2,
                                                             &CpscMotorCreateControllerArg3,
                                                             &CpscMotorCreateControllerArg4};
static const iocshFuncDef CpscMotorCreateControllerDef = {"CpscMotorCreateController", 5, CpscMotorCreateControllerArgs};

static void CpscMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    CpscMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void CpscMotorRegister(void) {
    iocshRegister(&CpscMotorCreateControllerDef, CpscMotorCreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(CpscMotorRegister);
}



