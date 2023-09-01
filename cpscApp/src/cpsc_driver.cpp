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

/** Code for iocsh registration */
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



