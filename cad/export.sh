#!/bin/sh
# Exports each part to stl/ (binary STL) and the back plate to DXF.
set -e
cd "$(dirname "$0")"
mkdir -p stl
for p in cradle clamp_bar receiver jaw knob desk_base mast_foot head_fixed head_moving box lid \
         rail_coupon receiver_coupon; do
  openscad -q --export-format binstl -D "PART=\"$p\"" -o "stl/$p.stl" mounts.scad
  echo "stl/$p.stl"
done
openscad -q -D 'PART="back_plate"' -o back_plate.dxf mounts.scad && echo back_plate.dxf
