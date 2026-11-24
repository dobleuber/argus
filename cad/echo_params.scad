// Prints the dimensions the simulation uses (Python reads this output).
include <mounts.scad>
PART = "none";
echo(str("P:HINGE=", HINGE));
echo(str("P:PLATE_Z=", PLATE_Z));
echo(str("P:RX_ORIGIN=", RX_ORIGIN));
echo(str("P:ARM_BASE=", ARM_BASE));
echo(str("P:FOOT_AT=", FOOT_AT));
echo(str("P:HEAD_AT=", HEAD_AT));
echo(str("P:FOLD_TORQUE=", FOLD_TORQUE));
echo(str("P:SHEET_X=", SHEET_X));
echo(str("P:SHEET=", SHEET));
echo(str("P:SHEET_TOP=", SHEET_TOP));
echo(str("P:BOX_Z=", BOX_Z));
echo(str("P:BOX_IN=", BOX_IN));
