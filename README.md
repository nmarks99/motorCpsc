# JPE Cryo Positioning Systems Controller (CPSC)
[JPE CPSC](https://www.jpe-innovations.com/cryo-uhv-products/cryo-positioning-systems-controller/)

- Only closed loop motion using the controller's internal control loop is supported through the motor record.
- Open loop moves supported through additional records provided by `cpsc_axis.db` and `cpsc_controller.db`
- The motor record's NTM field must be set to 0 for reliable motion/moving detection. This is done for you
if you use the provided `cpsc_asyn_motor.db` instead of the usual `asyn_motor.db` from `motor`, or you can
alternatively add a `dbpf $(P)m$(N).NTM 0` for each motor after iocInit.
