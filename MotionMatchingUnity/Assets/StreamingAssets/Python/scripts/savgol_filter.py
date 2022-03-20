import numpy as np
import scipy.signal as signal

# From C#:
# 3 lists (pos_x, pos_y, pos_z) corresponding to the position of the simulation bone
# 3 lists (dir_x, dir_y, dir_z) corresponding to the directions of the simulation bone
# To C#:
# 3 lists (pos_f_x, pos_f_y, pos_f_z) corresponding to the filtered position of the simulation bone
# 3 lists (dir_f_x, dir_f_y, dir_f_z) corresponding to the filtered directions (NOT NORMALIZED) of the simulation bone

# Smooth positions
pos = np.array([pos_x, pos_y, pos_z]).transpose()
pos_filtered = signal.savgol_filter(
    pos, window_length=31, polyorder=3, axis=0, mode="interp"
)
pos_f_x = pos_filtered[:, 0].tolist()  # numpy to python list (so C# can read it)
pos_f_y = pos_filtered[:, 1].tolist()
pos_f_z = pos_filtered[:, 2].tolist()

# Smooth rotations
dir = np.array([dir_x, dir_y, dir_z]).transpose()
dir_filtered = signal.savgol_filter(
    dir, window_length=61, polyorder=3, axis=0, mode="interp"
)
dir_f_x = dir_filtered[:, 0].tolist()  # numpy to python list (so C# can read it)
dir_f_y = dir_filtered[:, 1].tolist()
dir_f_z = dir_filtered[:, 2].tolist()
