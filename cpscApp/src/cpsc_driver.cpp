#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>

#include "cpsc_driver.hpp"


// ===================
// CpscMotorController
// ===================

#define NUM_PARAMS 0  

CpscMotorController::CpscMotorController(
    const char *portName,
    const char *CpscMotorPortName,
    int numAxes,
    double movingPollPeriod,
    double idlePollPeriod
) : asynMotorController(portName, numAxes, NUM_PARAMS,
                        0,
                        0,
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                        1,
                        0, 0)
{
    asynStatus status;
    int axis;
    CpscMotorAxis *pAxis;
    static const char *functionName = "CpscMotorController::CpscMotorController";

    status = pasynOctetSyncIO->connect(CpscMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s: cannot connect to CPSC motor controller\n",
        functionName);
    }

    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new CpscMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int CpscMotorCreateController(
    const char *portName,
    const char *CpscMotorPortName,
    int numAxes,
    int movingPollPeriod,
    int idlePollPeriod)
{
    CpscMotorController *pCpscMotorController = new CpscMotorController(
        portName,
        CpscMotorPortName,
        numAxes,
        movingPollPeriod/1000.,
        idlePollPeriod/1000.
    );
    pCpscMotorController = NULL;
    return(asynSuccess);

}

void CpscMotorController::report(FILE *fp, int level) {
    fprintf(fp, "CPSC Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    /*
    * It is a good idea to print private variables that were added to the VirtualMotorController class in VirtualMotorDriver.h, here
    * This allows you to see what is going on by running the "dbior" command from iocsh.
    */

    // Call the base class method
    asynMotorController::report(fp, level);
}

CpscMotorAxis* CpscMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(pasynUser));
}

CpscMotorAxis* CpscMotorController::getAxis(int axisNo) {
    return static_cast<CpscMotorAxis*>(asynMotorController::getAxis(axisNo));
}


// =============
// CpscMotorAxis
// =============

CpscMotorAxis::CpscMotorAxis(CpscMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC)
{
    axisIndex_ = axisNo + 1;
    setDoubleParam(pC_->motorEncoderPosition_, 0.0);
    callParamCallbacks();
}

void CpscMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
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



