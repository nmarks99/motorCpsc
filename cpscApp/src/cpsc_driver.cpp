#include <math.h>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <map>
#include <random>
#include <iostream>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

#include "cpsc_driver.hpp"

// readback is in meters with nanometer precision
const long int PREC = 9; 

double gen_random_float(double min, double max) {
    std::random_device rd;  // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator

    std::uniform_real_distribution<> distribution(min, max); // define the range

    return distribution(eng); // generate the random number
}

enum Color {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    RESET
};

std::string stylize_string(std::string s, Color color) {
    std::map<Color, std::string> color_map = {
        {RED, "\x1b[31m"},
        {GREEN, "\x1B[32m"},
        {BLUE, "\x1B[34m"},
        {YELLOW, "\x1B[33m"},
        {RESET,"\x1b[0m"},
    };

    std::stringstream ss;
    ss << color_map[color];
    ss << s;
    ss << color_map[Color::RESET];
    return ss.str();
}

// splits a char array into a std::vector<double>
std::vector<double> split_char_arr(const char *msg, char delimiter=',') {
    // copies char* to std::string, replaces delimiter with spaces,
    // creates an istringstream from the string and splits by whitespace,
    // and finally, converts each substring to a double and stores it in a
    // std::vector<double> and returns it
    std::string s_msg(msg);
    std::replace(s_msg.begin(), s_msg.end(), delimiter, ' ');
    std::istringstream ss(s_msg);
    std::vector<double> v_out{std::istream_iterator<double>(ss), {}};
    return v_out;
}


// ===================
// CpscMotorController
// ===================

static const int NUM_PARAMS = 0;

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
    
    // only feedback for 3 axes I think
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
    
    // additional controller (not axis specific) initialization:

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
    setDoubleParam(pC_->motorEncoderPosition_, 0.0);
    setDoubleParam(pC_->motorPosition_, 0.0);
    setDoubleParam(pC_->motorResolution_, 1.0);

    // enables setClosedLoop function:
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
    once = true;

    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "CpscMotorAxis created with axis index: %d\n", axisIndex_);
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
    // sprintf(pC_->outString_, "STP %d", axisIndex_);
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBES\n");
    sprintf(pC_->outString_, "FBES"); // Feedback mode E-stop
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

    // sets the setpoint for the current axis to "position" absolute
    // other axes are set to move 0.0 relative to their current position
    switch (axisIndex_) {
        case 1:
            asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS %lf 1 0 0 0 0\n", position);
            // sprintf(pC_->outString_, "FBCS %lf 1 0 0 0 0", position);
            break;
        case 2: 
            asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS 0 0 %lf 1 0 0\n", position);
            // sprintf(pC_->outString_, "FBCS 0 0 %lf 1 0 0", position);
            break;
        case 3: 
            asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "FBCS 0 0 0 0 %lf 1\n", position);
            // sprintf(pC_->outString_, "FBCS 0 0 0 0 %lf 1", position);
            break;
        default:
            asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Invalid axis index %d\n", axisIndex_);
    }
    // status = pC_->writeReadController();
    return status;
}


asynStatus CpscMotorAxis::poll(bool *moving) {
    asynStatus asyn_status;
    double position;
    int done;
    std::vector<double> status;
    
    // Read position
    sprintf(pC_->outString_, "PGV 4 %d CBS10-RLS", axisIndex_);
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Error reading position\n");
        setIntegerParam(pC_->motorStatusProblem_, asyn_status);
        callParamCallbacks();
        return asyn_status ? asynError : asynSuccess;
    }

    // Adjust MRES to obtain the desired units of the readback values
    // MRES = 1.0 -> nanometers
    // MRES = 1e-3 -> micrometers
    // MRES = 1e-6 -> millimeters
    // MRES = 1e-9 -> meters
    position = atof((const char *) &pC_->inString_);
    long long_position = position * pow(10, PREC);
    setDoubleParam(pC_->motorPosition_, long_position);
    
    // Read status 
    // [ENABLED] [FINISHED] [INVALID SP1] [INVALID SP2] [INVALID SP3] [POS ERROR1] [POS ERROR2] [POS ERROR3]
    sprintf(pC_->outString_, "FBST");
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Error reading status\n");
        setIntegerParam(pC_->motorStatusProblem_, asyn_status);
        callParamCallbacks();
        return asyn_status ? asynError : asynSuccess;
    }

    // split input char* by ',' into a std::vector<double>
    status = split_char_arr(pC_->inString_, ',');
    
    // get done status
    done = status.at(1);
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);
    *moving = !status.at(1);
   
    // Handle error codes
    if (!status.at(0)) {
        if (once) {
            asynPrint(
                pasynUser_,
                ASYN_TRACE_ERROR,
                stylize_string("Error(%d): Feedback mode is disabled\n", Color::RED).c_str(),
                axisIndex_
            );
            once = false;
        }
    }
    if (int(status.at(2)) == 1) {
         asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Error: Invalid setpoint on axis 1\n");
    }
    if (int(status.at(3)) == 1) {
         asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Error: Invalid setpoint on axis 2\n");
    }
    if (int(status.at(4)) == 1) {
         asynPrint(pasynUser_, ASYN_TRACE_ERROR, "Error: Invalid setpoint on axis 3\n");
    }
    
    // skip:
    setIntegerParam(pC_->motorStatusProblem_, asyn_status);
    callParamCallbacks();
    return asyn_status ? asynError : asynSuccess;
}


/// \brief Enable closed loop (Servodrive mode)
asynStatus CpscMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus status;

    if (closedLoop) {
        // enable closed loop
        // add check for whether or not controller is already in closed loop mode
        asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Feedback mode enabled\r\n");
        sprintf(pC_->outString_, "FBEN CBS10-RLS 300 CBS10-RLS 300 CBS10-RLS 300 1 293");

    }
    else {
        // disable closed loop
        asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "Feedback mode disabled\r\n");
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



