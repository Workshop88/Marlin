/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * motion.h
 *
 * High-level motion commands to feed the planner
 * Some of these methods may migrate to the planner class.
 */

#ifndef MOTION_H
#define MOTION_H

#include "../inc/MarlinConfig.h"

#include "../module/planner.h"

extern float feedrate_mm_s;
extern float current_position[XYZE], destination[XYZE], target[XYZE];

extern uint8_t active_extruder;

#if HAS_SOFTWARE_ENDSTOPS
  extern bool soft_endstops_enabled;
#endif
extern float soft_endstop_min[XYZ], soft_endstop_max[XYZ];

inline void set_current_to_destination() { COPY(current_position, destination); }
inline void set_destination_to_current() { COPY(destination, current_position); }

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
inline void line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_s); }

#if IS_KINEMATIC
  void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);
#endif // IS_KINEMATIC

void prepare_move_to_destination();

void clamp_to_software_endstops(float target[XYZ]);

#if ENABLED(MESH_BED_LEVELING)
  void mesh_line_to_destination(const float fr_mm_s, uint8_t x_splits=0xFF, uint8_t y_splits=0xFF);
#elif ENABLED(AUTO_BED_LEVELING_BILINEAR) && !IS_KINEMATIC
  void bilinear_line_to_destination(const float fr_mm_s, uint16_t x_splits=0xFFFF, uint16_t y_splits=0xFFFF);
#endif

#if IS_KINEMATIC && !UBL_DELTA
  bool prepare_kinematic_move_to(float ltarget[XYZE]);
#else
  bool prepare_move_to_destination_cartesian();
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  bool prepare_move_to_destination_dualx();
#endif

#endif // MOTION_H
