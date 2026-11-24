// Dimensions of the SO-101 arm mount system (mm). Single source: change them here and export
// again (./export.sh). The simulation reads the backpack ones through echo_params.scad.

$fn = 48;
CL = 0.4;            // fit clearance (lower to 0.3 if the printer is accurate)
WALL = 4;            // structural walls

// ---------------------------------------------------------------- SO-101 base (measured from the STL)
// Base flange: 87 x 111 mm, 16 mm tall; the tower on top spans x -22.4..19.4, y ±22.4.
// Coordinates relative to the origin of the model's "base" body (x axis towards the front of the arm).
BASE_X0 = -22.4;  BASE_L = 87.0;  BASE_W = 111.0;  BASE_H = 16.0;
BASE_Z0 = -2.4;   // the bottom face of the base is 2.4 mm below its origin
TOWER_X1 = 19.4;  TOWER_HW = 22.4;

// ---------------------------------------------------------------- cradle (hugs the flange)
FLOOR = 4;          // floor under the base
TRAY_WALL_H = 10;   // height of the side walls (the flange is 16)
BAR_W = 20;  BAR_T = 8;  BAR_Y = 39;   // clamp bars over the wings (y = ±39)
BAR_EAR = 10;       // the bars stick out and are bolted outside the flange
PAD = 1.0;          // rubber or TPU under the bars

// ---------------------------------------------------------------- dovetail rail (Arca-type, 38 mm)
// Profile (bottom to top): 38 mm wide for the first RAIL_FLAT mm, then it narrows at 45° to
// RAIL_W - 2*RAIL_BEVEL. The receiver jaws ride on those bevels and pull the rail down.
// Print the coupons first (PART="rail_coupon" / "receiver_coupon") and test them with the clamp you will use.
RAIL_W = 38;  RAIL_FLAT = 6;  RAIL_BEVEL = 3;  RAIL_L = 100;
RAIL_H = RAIL_FLAT + RAIL_BEVEL;
PIN_GROOVE_W = 6.2;  PIN_GROOVE_D = 2;   // groove for the M4 safety screw

// ---------------------------------------------------------------- receiver (desk and backpack)
RX_L = 70;          // clamping length along the rail
RX_SIDE = 16;       // width of each jaw
RX_BASE = 8;        // thickness under the rail
JAW_TRAVEL = 4;     // travel of the moving jaw

// ---------------------------------------------------------------- hardware
M3_INSERT = 4.0;  M5 = 5.4;  M5_HEAD = 9.5;  M5_NUT = 8.0 / cos(30) + 0.4;  M5_NUT_H = 4.2;
M6 = 6.4;  M6_NUT = 10.0 / cos(30) + 0.4;  M6_NUT_H = 5.2;  M4 = 4.4;  M8 = 8.4;

// ---------------------------------------------------------------- backpack (frame: wearer's shoulders)
// x forward, y left, z up; origin = centre of the shoulders. The arm sits on the right shoulder.
ARM_BASE = [-100, -260, 0];      // origin of the arm base (same as MOUNT in sim/backpack.py)
SHEET = [3, 250, 380];           // 3 mm HDPE plate (fits in the laptop pocket)
SHEET_X = -135;                  // front face of the plate (towards the back)
SHEET_TOP = -40;                 // top edge of the plate
TUBE = 20;  TUBE_T = 1.5;        // 20 x 20 x 1.5 square aluminium tube
FOOT_AT = [-154, -100, -60];     // where the tube starts (foot bolted to the top right of the plate)
HEAD_AT = [-154, -225, -45];     // where the tube ends (head with hinge)
HINGE_BEHIND = 30;               // the hinge is behind the rail: the arm folds backwards
FOLD_TORQUE = 7;                 // N·m: beyond this, the magnets give way and the arm folds
MAGNET_D = 10;  MAGNET_H = 5;    // 10 x 5 mm neodymium magnets (4 pairs)
// Support of the fixed head (improved after the generative-design trial, ../generative/): thinner
// plate + ribs underneath + a wall that ties them to the tube socket
SUPPORT_T = 6;                   // support thickness (was 10)
MAG_BOSS_D = 16;  MAG_BOSS_H = 9;   // round boss around each magnet
RIB_T = 4;  RIB_DEPTH = 28;      // ribs under the support: thickness and height at the root (28: stays inside the original bounding box)
RIB_Y = [-33, 0, 33];            // position of the ribs relative to the centre of the arm
BOX_IN = [45, 140, 95];          // electronics box (inside): Orange Pi 5 Pro + 3S battery + buck
BOX_Z = -330;                    // centre of the box (low on the plate)
