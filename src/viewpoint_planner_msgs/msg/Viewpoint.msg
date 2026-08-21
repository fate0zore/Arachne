# A single planned viewpoint.
builtin_interfaces/Time stamp  # planning time, for aligning multiple calls
float64[3] camera_xyz_base     # desired camera position, base_link frame
float64[4] camera_quat_xyzw    # desired camera orientation, base_link frame (optical axis toward target)
float64[3] tool0_xyz_base      # converted tool0 position, base_link frame
float64[4] tool0_quat_xyzw     # converted tool0 orientation, base_link frame
bool ik_reachable              # IK reachability flag (still marked per-point when filtering is off)
float64[6] ik_joints           # joint solution when reachable (aubo joint angles; zeros when unreachable)
bool collision_free            # collision-free flag (after IK; still marked per-point when filtering is off; false when no valid q)
float64 collision_clearance    # minimum clearance (m) to obstacles/ground/self; negative = penetration
