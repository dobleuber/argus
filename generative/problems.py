"""Generative design of the fixed head of the backpack mount (trial).

Part frame (mm, N): X = forward of the wearer, from the hinge axis; Y = DOWNWARDS, from the
bearing face of the moving plate (Y < 0 is where the moving part moves); Z = towards the
wearer's left, from the outer face on the right side. Z = 0 is the print bed.
The hinge axis (M8 pin) and the aluminium tube run along Z.

For the trial the tube is horizontal (in the design it rises 15 mm over 125 mm, ~7°): that way the
tube and the pin are parallel and all the important holes run along Z. The original part used for
comparison (baselines/head_fixed.stl) is the same head_fixed with the tube horizontal
(head_fixed_level.scad).

Loads (from the backpack simulation, sim/demo_walk.py), with a 1.5 margin:
  - walking while holding 500 g: the mount takes ~16 N and ~3.3 N·m -> 24 N downwards, 206 mm
    in front of the arm base (3.3 / 16) and 100 mm up (centre of mass of the arm);
  - the same with a sideways push of half that, to each side (the sway of the gait);
  - the branch: the gripper pushed backwards with 20 N (the most before the magnets give way:
    7 N·m / 0.35 m), plus the weight of the arm;
  - the magnets about to give way: they pull the bearing face up with ~100 N (7 N·m at ~70 mm
    from the hinge) and the pin pushes down by the same amount.
"""
from genpart import Problem, RemoteLoad, box, zcyl

# Dimensions (from cad/params.scad and mounts.scad, converted to this frame)
LUG_Z = (14.4, 55.6)          # central (fixed) knuckle of the hinge, r = 11
KR = 11.0
SUPPORT_X = (14.0, 105.0)     # support under the moving plate
SUPPORT_Z = (0.0, 70.0)
MAGNETS = [(55.0, 9.0), (85.0, 9.0), (55.0, 61.0), (85.0, 61.0)]   # (X, Z), 10.3 x 5.2 pockets from Y = 0
RX_BOLTS = [(40.0, 6.0), (40.0, 64.0), (95.0, 6.0), (95.0, 64.0)]  # recesses for the receiver bolts
TUBE = (-5.1, 28.6)           # tube centre (X, Y)
SOCKET_Z = (62.0, 107.0)      # the tube goes in up to Z = 70; the socket runs from 62 to 107
CROSS_Z = (74.0, 95.0)        # M5 cross bolts through the socket (along Y)

# Points of application (part frame)
ARM_LOAD_POINT = (255.0, -131.0, 35.0)     # 206 mm in front of the arm base, 100 mm up
BRANCH_POINT = (305.0, -351.0, 35.0)       # the gripper, 1.77 m above the ground


def head_fixed():
    lug = zcyl(0, 0, KR, LUG_Z[0], LUG_Z[1])
    skin = box(SUPPORT_X[0], SUPPORT_X[1], 0, 3, SUPPORT_Z[0], SUPPORT_Z[1])   # flat bearing face
    # The arm loads enter through the hinge and through the bearing face; each split into two
    # halves (in Z) so that they can carry moments
    mid = 35.0
    arm_patches = [zcyl(0, 0, KR, LUG_Z[0], mid), zcyl(0, 0, KR, mid, LUG_Z[1]),
                   box(SUPPORT_X[0], SUPPORT_X[1], 0, 3, SUPPORT_Z[0], mid),
                   box(SUPPORT_X[0], SUPPORT_X[1], 0, 3, mid, SUPPORT_Z[1])]
    bosses = [box(x - 8, x + 8, 0, 8, z - 8, z + 8) for x, z in MAGNETS]      # around the magnets
    tx, ty = TUBE
    socket = box(tx - 15, tx + 15, ty - 15, ty + 15, SOCKET_Z[0], SOCKET_Z[1])
    return Problem(
        name="head_fixed",
        # Space: up to 1 cm further back than the original and 1.2 cm lower; the shoulder limits it at the front
        bounds=(-30, 105, -11, 55, 0, 107),
        keepout=[
            # Above the bearing face the moving part moves (and folds backwards): all free,
            # except the central knuckle (which fits into the gap in the moving plate, radius 12)
            box(-99, 99, -99, 0, -99, LUG_Z[0] - 0.2),
            box(-99, 99, -99, 0, LUG_Z[1] + 0.2, 99),
            box(8.5, 99, -99, 0, LUG_Z[0] - 1, LUG_Z[1] + 1),
            box(-99, -8.5, -99, 0, LUG_Z[0] - 1, LUG_Z[1] + 1),
            box(-99, 99, -99, -8.5, LUG_Z[0] - 1, LUG_Z[1] + 1),
            # Moving knuckles on either side of the fixed one (they rotate: full cylinder of radius 12.5)
            zcyl(0, 0, 12.5, -99, LUG_Z[0] - 0.2),
            zcyl(0, 0, 12.5, LUG_Z[1] + 0.2, 99),
            # The aluminium tube (20 x 20 + clearance), from Z = 70 towards the foot
            box(tx - 10.4, tx + 10.4, ty - 10.4, ty + 10.4, SOCKET_Z[0], 99),
            # Heads and nuts of the socket M5 cross bolts (top and bottom) and the wrench
            *[box(tx - 6, tx + 6, -99, ty - 15, z - 6, z + 6) for z in CROSS_Z],
            *[box(tx - 6, tx + 6, ty + 15, 99, z - 6, z + 6) for z in CROSS_Z],
            # Recesses for the receiver bolt heads (in the moving plate)
            *[box(x - 6, x + 6, -1, 4.5, z - 6, z + 6) for x, z in RX_BOLTS],
        ],
        preserve=[lug, skin, *bosses, socket],
        holes=[
            zcyl(0, 0, 4.2, -99, 99),                                                  # M8 pin
            *[box(x - 5.2, x + 5.2, -1, 5.2, z - 5.2, z + 5.2) for x, z in MAGNETS],   # 10 x 5 magnets
            *[box(tx - 2.7, tx + 2.7, -99, 99, z - 2.7, z + 2.7) for z in CROSS_Z],    # M5 cross bolts
        ],
        support=[box(tx - 15, tx + 15, ty - 15, ty + 15, 75, 107)],   # the tube holds it
        cases={
            "walk with 500 g": [RemoteLoad(arm_patches, ARM_LOAD_POINT, (0, 24, 0))],
            "walk, push to the left": [RemoteLoad(arm_patches, ARM_LOAD_POINT, (0, 24, 12))],
            "walk, push to the right": [RemoteLoad(arm_patches, ARM_LOAD_POINT, (0, 24, -12))],
            "branch (backwards)": [RemoteLoad(arm_patches, BRANCH_POINT, (-20, 10, 0))],
            "magnets about to give way": [RemoteLoad(bosses, (70, 0, 35), (0, -100, 0)),
                                        RemoteLoad([lug], (0, 0, 35), (0, 100, 0))],
        },
        # volfrac: attempt 1 = 0.155 (the same mass as the original); attempt 2 = 0.125 (~20 % less;
        # below 0.12 is not possible: the preserved regions already take 0.07)
        round=2.0, voxel=3.0, volfrac=0.125, rmin=2.5, min_wall=1.2,
        youngs=2100.0, density=1.27e-3,   # PETG
        baseline="baselines/head_fixed.stl",
    )


def head_fixed_ribs():
    """The original improved by hand with what the generative design showed (ribs under the support
    and a wall that ties them to the socket; 6 mm support). Same problem and same loads: only to
    evaluate it against the original (out/head_fixed_ribs.stl comes from head_fixed_level.scad)."""
    from dataclasses import replace
    return replace(head_fixed(), name="head_fixed_ribs")


PROBLEMS = {"head_fixed": head_fixed, "head_fixed_ribs": head_fixed_ribs}
