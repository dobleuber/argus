// Fixed head of the backpack mount, with the tube HORIZONTAL (only for the generative-design
// trial: the hinge pin and the tube are parallel, along Z).
// Part frame: X = wearer's x (forward) from the hinge axis, Y = downwards from the bearing face
// of the moving plate, Z = wearer's y (to the left) from the outer face. Z up is up from the
// print bed.
include <../cad/mounts.scad>
PART = "none";
HEAD_AT = [-154, -225, -60];   // horizontal tube (in the design: -45)
Y_MIN = ARM_BASE[1] - RX_W / 2;   // outer face (right side)
echo(str("G:Y_MIN=", Y_MIN, " HINGE=", HINGE, " PLATE_Z=", PLATE_Z, " RX_ORIGIN=", RX_ORIGIN, " TUBE_LEN=", TUBE_LEN));
multmatrix([[1, 0, 0, -HINGE[0]], [0, 0, -1, PLATE_Z], [0, 1, 0, -Y_MIN], [0, 0, 0, 1]]) head_fixed();
