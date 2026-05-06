# ../../bin/${EPICS_HOST_ARCH}/xeryonExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/ioccpscExampleLinux.dbd")
ioccpscExampleLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "cpscExample:")

< cpsc.iocsh

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
