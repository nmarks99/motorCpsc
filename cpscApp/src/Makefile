TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += cpsc

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install cpsc.dbd into <top>/dbd
DBD += cpscSupport.dbd

cpscSupport_DBD += cpsc.dbd

# specify all source files to be compiled and added to the library
cpsc_SRCS += cpsc_driver.cpp utils.cpp

cpsc_LIBS += motor asyn
cpsc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

