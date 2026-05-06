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

## Units
Adjust MRES to set the desired units for readback and commanded positions.

| MRES   | EGUs    |
|--------------- | --------------- |
| 1.0   | nanometers   |
| 1e-3   | micrometers   |
| 1e-6   | millimeters   |
| 1e-9   | meters   |

