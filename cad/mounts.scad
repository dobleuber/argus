// SO-101 arm mount system: the same joint (dovetail rail) works on the desk and on the backpack.
// Choose the part with PART (see export.sh), or view the assembly with PART="desk" / "backpack".
//
// Parts to print (PETG, 6 perimeters, 40 % infill):
//   cradle, clamp_bar (x2), receiver, jaw, knob, desk_base, mast_foot, head_fixed,
//   head_moving, box, lid, rail_coupon, receiver_coupon
// To buy: 20x20x1.5 aluminium tube, 3 mm HDPE plate (cut with back_plate.dxf),
//   M4/M5/M8 screws, nuts, 8 neodymium magnets 10x5 mm, C-clamps for the desk.

include <params.scad>

PART = "backpack";

XC = BASE_X0 + BASE_L / 2;                     // centre of the base flange
FLOOR_Z = BASE_Z0 - FLOOR;                     // bottom face of the cradle
RAIL_Z = FLOOR_Z - RAIL_H;                     // bottom face of the rail
BOLT_X = [BASE_X0 - BAR_EAR / 2 - 3, BASE_X0 + BASE_L + BAR_EAR / 2 + 3];

module along_x(L) { rotate([90, 0, 90]) linear_extrude(L) children(); }   // 2D (y, z) -> extruded along x

module rail_2d(grow = 0) {
    w = RAIL_W / 2 + grow;  n = RAIL_W / 2 - RAIL_BEVEL + grow;
    polygon([[-w, -grow], [w, -grow], [w, RAIL_FLAT], [n, RAIL_H + grow], [-n, RAIL_H + grow], [-w, RAIL_FLAT]]);
}

// ------------------------------------------------------------------------------------ cradle
module cradle() {
    ix = BASE_L + 2 * CL;  iy = BASE_W + 2 * CL;
    x0 = BOLT_X[0] - BAR_EAR;  x1 = BOLT_X[1] + BAR_EAR;
    difference() {
        union() {
            // floor + walls
            translate([x0, -iy / 2 - WALL, FLOOR_Z]) cube([x1 - x0, iy + 2 * WALL, FLOOR]);
            difference() {
                translate([BASE_X0 - CL - WALL, -iy / 2 - WALL, FLOOR_Z])
                    cube([ix + 2 * WALL, iy + 2 * WALL, FLOOR + TRAY_WALL_H]);
                translate([BASE_X0 - CL, -iy / 2, BASE_Z0]) cube([ix, iy, 50]);
            }
            // posts where the bars are bolted (at flange height)
            for (bx = BOLT_X, s = [-1, 1])
                translate([bx - BAR_EAR / 2 - 2, s * BAR_Y - BAR_W / 2, FLOOR_Z])
                    cube([BAR_EAR + 4, BAR_W, FLOOR + BASE_H - PAD]);
            // rail underneath
            translate([XC - RAIL_L / 2, 0, RAIL_Z]) along_x(RAIL_L) rail_2d();
        }
        // M5 bolts for the bars, with nut trapped from below
        for (bx = BOLT_X, s = [-1, 1]) translate([bx, s * BAR_Y, FLOOR_Z - 1]) {
            cylinder(d = M5, h = 40);
            cylinder(d = M5_NUT, h = M5_NUT_H + 1, $fn = 6);
        }
        // safety-screw groove (in the centre of the rail)
        translate([XC - PIN_GROOVE_W / 2, -RAIL_W, RAIL_Z - 1]) cube([PIN_GROOVE_W, 2 * RAIL_W, PIN_GROOVE_D + 1]);
        // floor cut-out under the tower (saves plastic)
        translate([XC, 0, FLOOR_Z + FLOOR / 2]) cube([40, 60, FLOOR + 0.1], center = true);
    }
}

module clamp_bar() {   // bar that clamps one wing of the flange (with PAD mm of rubber underneath)
    len = BOLT_X[1] - BOLT_X[0] + BAR_EAR + 4;
    difference() {
        translate([BOLT_X[0] - BAR_EAR / 2 - 2, -BAR_W / 2, 0]) cube([len, BAR_W, BAR_T]);
        for (bx = BOLT_X) translate([bx, 0, -1]) { cylinder(d = M5, h = 20); translate([0, 0, BAR_T - 3]) cylinder(d = M5_HEAD + 0.6, h = 5); }
    }
}

// ------------------------------------------------------------------------------------ receiver
// Coordinates: x along the rail (it slides in from x = 0 up to the stop at x = RX_L), z = 0 bottom
// face of the rail. The -y side is fixed; the +y side is a jaw pushed by an M5 screw.
RX_W = RAIL_W + 2 * RX_SIDE;
POCKET_Y0 = RAIL_W / 2 - RAIL_BEVEL - 1;        // where the jaw pocket starts
POCKET_Y1 = RX_W / 2 - 4;                        // 4 mm outer wall (holds the nut)
STOP = 5;                                         // end stop

module receiver_body() {
    difference() {
        translate([0, -RX_W / 2, -RX_BASE]) cube([RX_L + STOP, RX_W, RX_BASE + RAIL_H]);
        translate([-1, 0, 0]) along_x(RX_L + 1) rail_2d(CL);
        translate([-1, -(RAIL_W / 2 - RAIL_BEVEL + CL), RAIL_FLAT]) cube([RX_L + 1, RAIL_W - 2 * RAIL_BEVEL + 2 * CL, 20]);
        // pocket for the moving jaw
        translate([-1, POCKET_Y0, 0]) cube([RX_L + 1, POCKET_Y1 - POCKET_Y0, RAIL_H + 1]);
        // knob screw (M5 with nut trapped in the outer wall)
        translate([RX_L / 2, RX_W / 2 + 1, RAIL_H / 2]) rotate([90, 0, 0]) {
            cylinder(d = M5, h = 10);
            translate([0, 0, 1.2]) rotate([0, 0, 30]) cylinder(d = M5_NUT, h = M5_NUT_H, $fn = 6);
        }
        // M4 safety screw from below (goes into the rail groove)
        translate([RX_L / 2, 0, -RX_BASE - 1]) cylinder(d = M4 - 0.6, h = RX_BASE + 2);   // thread in the plastic or an insert
        // M5 holes to fix the receiver (to the desk base or to the mast head)
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([x, s * (RX_W / 2 - 6), -RX_BASE - 1]) {
            cylinder(d = M5, h = 30);
            translate([0, 0, RX_BASE - 2]) cylinder(d = M5_HEAD + 0.6, h = 30);
        }
    }
}

module jaw() {   // moving jaw: its inner face is the mirror bevel of the rail
    difference() {
        translate([0, POCKET_Y0 + CL, CL]) cube([RX_L - 2 * CL, POCKET_Y1 - POCKET_Y0 - 2 * CL - JAW_TRAVEL, RAIL_H - 2 * CL]);
        translate([-1, 0, 0]) along_x(RX_L + 2) rail_2d(CL);
    }
}

module knob() {   // knob for the head of an M5 x 30 screw
    difference() {
        union() {
            cylinder(d = 26, h = 10);
            for (a = [0:60:359]) rotate([0, 0, a]) translate([13, 0, 0]) cylinder(d = 6, h = 10);
        }
        translate([0, 0, -1]) cylinder(d = M5, h = 20);
        translate([0, 0, 5]) cylinder(d = M5_HEAD / cos(30) * 0.95, h = 10, $fn = 6);
    }
}

// ------------------------------------------------------------------------------------ desk
module table_mount() {   // receiver + wings for two C-clamps (like the original SO-101)
    W = 150;  D = 90;  T = 8;
    difference() {
        union() {
            translate([-(W - RX_L - STOP) / 2, -D / 2, -RX_BASE - T]) cube([W, D, T]);
            for (s = [-1, 1]) hull() {   // ribs
                translate([-(W - RX_L - STOP) / 2, s * (D / 2 - 3) - 3, -RX_BASE - T]) cube([W, 6, T]);
                translate([0, s * (RX_W / 2) - 3, -RX_BASE - T]) cube([RX_L + STOP, 6, T + 6]);
            }
        }
        // slots for the clamps and holes for wood screws
        for (x = [-(W - RX_L - STOP) / 2 + 15, (W + RX_L + STOP) / 2 - 15]) {
            translate([x - 10, -D / 2 + 15, -50]) cube([20, D - 30, 100]);
        }
        for (x = [-(W - RX_L - STOP) / 2 + 6, (W + RX_L + STOP) / 2 - 6], s = [-1, 1])
            translate([x, s * (D / 2 - 8), -50]) cylinder(d = 4.5, h = 100);
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([x, s * (RX_W / 2 - 6), -50]) cylinder(d = M5, h = 100);
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([x, s * (RX_W / 2 - 6), -RX_BASE - T - 1]) cylinder(d = M5_NUT, h = M5_NUT_H + 1, $fn = 6);
        translate([RX_L / 2, 0, -50]) cylinder(d = 10, h = 100);   // access to the safety screw
    }
}

// ------------------------------------------------------------------------------------ coupons
module coupon_rail() { along_x(20) rail_2d(); }
module coupon_receiver() { intersection() { receiver_body(); translate([RX_L / 2 - 10, -50, -50]) cube([20, 100, 100]); } }

// ------------------------------------------------------------------------------------ backpack
function v_sub(a, b) = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
TUBE_DIR = v_sub(HEAD_AT, FOOT_AT);
TUBE_LEN = norm(TUBE_DIR);
// Angles to orient the tube (local +z axis) from FOOT_AT to HEAD_AT
T_YAW = atan2(TUBE_DIR[1], TUBE_DIR[0]);
T_PITCH = acos(TUBE_DIR[2] / TUBE_LEN);
module tube_frame() { translate(FOOT_AT) rotate([0, 0, T_YAW]) rotate([0, T_PITCH, 0]) children(); }

module tube() { tube_frame() difference() { translate([-TUBE / 2, -TUBE / 2, 0]) cube([TUBE, TUBE, TUBE_LEN]);
    translate([-TUBE / 2 + TUBE_T, -TUBE / 2 + TUBE_T, -1]) cube([TUBE - 2 * TUBE_T, TUBE - 2 * TUBE_T, TUBE_LEN + 2]); } }

SOCKET = 45;   // length of tube inside each part
module socket_outer() { translate([-TUBE / 2 - 5, -TUBE / 2 - 5, 0]) cube([TUBE + 10, TUBE + 10, SOCKET]); }
module socket_cut() {
    translate([-TUBE / 2 - CL, -TUBE / 2 - CL, -1]) cube([TUBE + 2 * CL, TUBE + 2 * CL, SOCKET + 2]);
    for (z = [12, SOCKET - 12]) translate([-30, 0, z]) rotate([0, 90, 0]) cylinder(d = M5, h = 60);   // M5 cross bolts
}

module mast_foot() {   // bolted (4 x M5) to the back face of the plate, top right
    difference() {
        hull() {
            translate([SHEET_X - SHEET[0] - 6, FOOT_AT[1] - 35, FOOT_AT[2] - 35]) cube([6, 70, 70]);
            tube_frame() translate([0, 0, -8]) socket_outer();
        }
        tube_frame() translate([0, 0, -8]) socket_cut();
        for (dy = [-25, 25], dz = [-25, 25]) translate([SHEET_X - 40, FOOT_AT[1] + dy, FOOT_AT[2] + dz])
            rotate([0, 90, 0]) cylinder(d = M5, h = 60);
    }
}

// Head: the fixed part grips the end of the tube and carries the hinge (M8 pin) and the stop; the
// moving part carries the receiver. When closed, 4 pairs of magnets hold it; if the arm gets snagged
// (more than FOLD_TORQUE), it folds backwards, never towards the wearer's head.
RX_TOP = ARM_BASE[2] + FLOOR_Z;                  // top face of the receiver (under the cradle)
RX_ORIGIN = [ARM_BASE[0] + XC - RX_L / 2 - STOP, ARM_BASE[1], RX_TOP - RAIL_H];
PLATE_T = 8;
PLATE_Z = RX_ORIGIN[2] - RX_BASE - PLATE_T;       // bottom face of the moving plate (= support)
HINGE = [RX_ORIGIN[0] - HINGE_BEHIND, ARM_BASE[1], PLATE_Z];
KNUCKLE = 14;  KR = 11;                           // width of each moving knuckle, and radius
SUPPORT_X0 = HINGE[0] + KR + 3;                   // the support starts after the hinge
SUPPORT_X1 = RX_ORIGIN[0] + RX_L + STOP;          // and ends with the receiver
SPINE_X0 = HEAD_AT[0] + TUBE / 2 + 5 - 1;         // wall: against the front face of the socket
SPINE_X1 = SPINE_X0 + 6;
SPINE_Y1 = HEAD_AT[1] + 30;                       // reaches the tube socket
MAG_X = [RX_ORIGIN[0] + 25, RX_ORIGIN[0] + 55];
MAG_Y = RX_W / 2 - 9;

module head_moving() {
    difference() {
        union() {
            translate(RX_ORIGIN) receiver_body();
            translate([HINGE[0], ARM_BASE[1] - RX_W / 2, PLATE_Z]) cube([RX_ORIGIN[0] + RX_L + STOP - HINGE[0], RX_W, PLATE_T]);
            for (s = [-1, 1]) translate([HINGE[0], ARM_BASE[1] + s * (RX_W - KNUCKLE) / 2, HINGE[2]])
                rotate([90, 0, 0]) cylinder(r = KR, h = KNUCKLE, center = true);
        }
        translate(HINGE) rotate([90, 0, 0]) cylinder(d = M8, h = 200, center = true);
        // gap for the central (fixed) knuckle, with room to rotate
        lug = RX_W - 2 * KNUCKLE;
        translate(HINGE) rotate([90, 0, 0]) cylinder(r = KR + 1, h = lug, center = true);
        translate([HINGE[0], ARM_BASE[1] - lug / 2, PLATE_Z - 1]) cube([KR + 1, lug, PLATE_T + 2]);
        for (x = MAG_X, s = [-1, 1]) translate([x, ARM_BASE[1] + s * MAG_Y, PLATE_Z - 0.01])
            cylinder(d = MAGNET_D + 0.3, h = MAGNET_H + 0.2);   // magnets (glued in with epoxy)
        // clearance holes for the receiver bolts
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([RX_ORIGIN[0] + x, ARM_BASE[1] + s * (RX_W / 2 - 6), PLATE_Z - 1])
            cylinder(d = M5, h = 30);
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([RX_ORIGIN[0] + x, ARM_BASE[1] + s * (RX_W / 2 - 6), PLATE_Z - 1])
            cylinder(d = M5_NUT, h = M5_NUT_H + 1, $fn = 6);
    }
}

module head_fixed() {
    lug = RX_W - 2 * KNUCKLE - 2 * CL;
    difference() {
        union() {
            // central knuckle + connection to the tube
            hull() {
                translate(HINGE) rotate([90, 0, 0]) cylinder(r = KR, h = lug, center = true);
                tube_frame() translate([0, 0, TUBE_LEN - SOCKET + 8]) socket_outer();
            }
            // support under the moving plate (carries the other magnets); the stop that prevents folding forwards
            hull() {
                translate([SUPPORT_X0, ARM_BASE[1] - RX_W / 2, PLATE_Z - SUPPORT_T]) cube([SUPPORT_X1 - SUPPORT_X0, RX_W, SUPPORT_T]);
                translate([HINGE[0], ARM_BASE[1] - lug / 2, PLATE_Z - KR]) cube([1, lug, 2]);
            }
            // bosses around the magnets
            for (x = MAG_X, s = [-1, 1]) translate([x, ARM_BASE[1] + s * MAG_Y, PLATE_Z - MAG_BOSS_H])
                cylinder(d = MAG_BOSS_D, h = MAG_BOSS_H);
            // wall in front of the tube socket: ties the ribs to the tube
            translate([SPINE_X0, ARM_BASE[1] - RX_W / 2, PLATE_Z - SUPPORT_T - RIB_DEPTH])
                cube([SPINE_X1 - SPINE_X0, SPINE_Y1 - (ARM_BASE[1] - RX_W / 2), RIB_DEPTH + SUPPORT_T]);
            // ribs under the support: tall at the root (wall) and tapering towards the front
            for (dy = RIB_Y) hull() {
                translate([SPINE_X0, ARM_BASE[1] + dy - RIB_T / 2, PLATE_Z - SUPPORT_T - RIB_DEPTH])
                    cube([SPINE_X1 - SPINE_X0 + 6, RIB_T, RIB_DEPTH + SUPPORT_T]);
                translate([SUPPORT_X1 - 8, ARM_BASE[1] + dy - RIB_T / 2, PLATE_Z - SUPPORT_T]) cube([1, RIB_T, SUPPORT_T]);
            }
        }
        tube_frame() translate([0, 0, TUBE_LEN - SOCKET + 8]) socket_cut();
        translate(HINGE) rotate([90, 0, 0]) cylinder(d = M8, h = 200, center = true);
        for (x = MAG_X, s = [-1, 1]) translate([x, ARM_BASE[1] + s * MAG_Y, PLATE_Z - MAGNET_H - 0.19])
            cylinder(d = MAGNET_D + 0.3, h = MAGNET_H + 0.2);
        // room for the moving knuckles to rotate
        for (s = [-1, 1]) translate([HINGE[0], ARM_BASE[1] + s * (RX_W - KNUCKLE) / 2, HINGE[2]])
            rotate([90, 0, 0]) cylinder(r = KR + 1, h = KNUCKLE + 2 * CL, center = true);
        // the receiver bolt heads sit in the moving plate: recess in the support
        for (x = [10, RX_L - 5], s = [-1, 1]) translate([RX_ORIGIN[0] + x, ARM_BASE[1] + s * (RX_W / 2 - 6), PLATE_Z - 4])
            cylinder(d = M5_NUT + 2, h = 5);
    }
}

// ------------------------------------------------------------------------------------ electronics
module box() {   // box low on the plate: Orange Pi 5 Pro, 3S battery (3 x 18650), 5 V buck, stop relay
    w = 2.5;
    difference() {
        translate([-BOX_IN[0] / 2 - w, -BOX_IN[1] / 2 - w, -BOX_IN[2] / 2 - w]) cube([BOX_IN[0] + 2 * w, BOX_IN[1] + 2 * w, BOX_IN[2] + w]);
        translate([-BOX_IN[0] / 2, -BOX_IN[1] / 2, -BOX_IN[2] / 2]) cube([BOX_IN[0], BOX_IN[1], BOX_IN[2] + 1]);
        for (dy = [-50, 50], dz = [-30, 30]) translate([BOX_IN[0] / 2 - 1, dy, dz]) rotate([0, 90, 0]) cylinder(d = M5, h = 10);
        for (dy = [-40, 0, 40]) translate([0, dy, -BOX_IN[2] / 2 - 5]) cylinder(d = 12, h = 10);   // cable grommets
    }
}
module lid() { translate([-BOX_IN[0] / 2 - 2.5, -BOX_IN[1] / 2 - 2.5, 0]) cube([BOX_IN[0] + 5, BOX_IN[1] + 5, 2.5]); }

module sheet_2d() {   // back plate (3 mm HDPE), front view: (y, z)
    difference() {
        offset(r = 20) offset(delta = -20) translate([-SHEET[1] / 2, SHEET_TOP - SHEET[2]]) square([SHEET[1], SHEET[2]]);
        for (dy = [-25, 25], dz = [-25, 25]) translate([FOOT_AT[1] + dy, FOOT_AT[2] + dz]) circle(d = M5);
        for (dy = [-50, 50], dz = [-30, 30]) translate([dy, BOX_Z + dz]) circle(d = M5);
        for (s = [-1, 1], z = [SHEET_TOP - 30, SHEET_TOP - SHEET[2] + 30]) translate([s * (SHEET[1] / 2 - 15) - 2.5, z - 13]) square([5, 26]);   // straps
        // lightening windows (~150 g instead of ~270 g), away from the screws
        for (s = [-1, 1]) translate([s * 55, SHEET_TOP - 210]) offset(r = 15) offset(delta = -15) square([80, 170], center = true);
    }
}
module sheet() { translate([SHEET_X - SHEET[0], 0, 0]) rotate([90, 0, 90]) linear_extrude(SHEET[0]) sheet_2d(); }

// ------------------------------------------------------------------------------------ assemblies
module arm_ghost() { color([1, 0.82, 0.12, 0.5]) translate([0, 0, BASE_Z0]) {
    translate([BASE_X0, -BASE_W / 2, 0]) cube([BASE_L, BASE_W, BASE_H]);
    translate([BASE_X0, -TOWER_HW, BASE_H]) cube([TOWER_X1 - BASE_X0, 2 * TOWER_HW, 60]); } }

module mounted_arm() {
    color("gainsboro") cradle();
    color("dimgray") for (s = [-1, 1]) translate([0, s * BAR_Y, BASE_Z0 + BASE_H]) clamp_bar();
    arm_ghost();
}

if (PART == "cradle") cradle();
if (PART == "clamp_bar") clamp_bar();
if (PART == "receiver") receiver_body();
if (PART == "jaw") jaw();
if (PART == "knob") knob();
if (PART == "desk_base") union() { table_mount(); }
if (PART == "mast_foot") mast_foot();
if (PART == "head_fixed") head_fixed();
if (PART == "head_moving") head_moving();
if (PART == "box") box();
if (PART == "lid") lid();
if (PART == "rail_coupon") coupon_rail();
if (PART == "receiver_coupon") coupon_receiver();
if (PART == "back_plate") sheet_2d();
if (PART == "desk") {
    color("lightsteelblue") table_mount();
    color("gainsboro") receiver_body();
    color("dimgray") jaw();
    translate([RX_L + STOP - (XC + RAIL_L / 2) + 0, 0, -RAIL_Z]) mounted_arm();
}
if (PART == "backpack") {
    color([0.9, 0.9, 0.9, 0.8]) sheet();
    color("silver") tube();
    color("slategray") mast_foot();
    color("slategray") head_fixed();
    color("lightsteelblue") head_moving();
    translate(ARM_BASE) mounted_arm();
    color("darkolivegreen") translate([SHEET_X - SHEET[0] - BOX_IN[0] / 2 - 2.5, 0, BOX_Z]) box();
}
