# Example IOC for using the JPE Cryo Positioning Systems Controller (CPSC)

First, fill in the path to the CPSC support module in configure/RELEASE
```
CPSC=/PATH/TO/SUPPORT/jpe-cpsc/cpsc/
```

To start the IOC run the following:
```
cd ./iocBoot/ioccpscIOC/
../../bin/rhel9-x86_64/cpscIOC st.cmd.Linux
```
