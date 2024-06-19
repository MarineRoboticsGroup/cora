"""
A file to visualize the different experimental data sets in the examples folder.

NOTE: the odometry visualization/calibration is unrefined and may not work well
for all data sets. It is likely a 80% solution that may need some more work if
you want to really inspect your odometry data/calibration.
"""

from py_factor_graph.io.pyfg_text import read_from_pyfg_text
from py_factor_graph.calibrations.range_measurement_calibration import (
    calibrate_range_measures,
)
from py_factor_graph.calibrations.odom_measurement_calibration import (
    calibrate_odom_measures,
)
import os


def _get_pyfg_files_in_dir(dir_path):
    # search in all subdirectories as well
    pyfg_files = []
    for root, _, files in os.walk(dir_path):
        for file in files:
            if file.endswith(".pyfg"):
                pyfg_files.append(os.path.join(root, file))
    return pyfg_files


def _select_from_list_based_on_requested_input(list_of_items, message):
    print(message)
    for i, item in enumerate(list_of_items):
        print(f"{i+1}: {item}")
    selected_index = int(input("Enter the index of the item you want to select: "))
    return list_of_items[selected_index - 1]


def _visualize_dataset(
    fg, show_gt, show_range_lines, show_range_circles, num_timesteps_keep_ranges
):
    dim = fg.dimension
    if dim == 2:
        fg.animate_odometry(
            show_gt=show_gt,
            draw_range_lines=show_range_lines,
            draw_range_circles=show_range_circles,
            num_timesteps_keep_ranges=num_timesteps_keep_ranges,
        )
    elif dim == 3:
        fg.animate_odometry_3d(
            show_gt=show_gt,
            draw_range_lines=show_range_lines,
            num_timesteps_keep_ranges=num_timesteps_keep_ranges,
        )
    else:
        raise ValueError(
            f"Only 2D and 3D data sets are supported. Received {dim}D data set."
        )


def _visualize_range_errors(fg):
    calibrate_range_measures(fg, show_outlier_rejection=True)


def _visualize_relative_pose_errors(fg):
    calibrate_odom_measures(fg)


if __name__ == "__main__":
    SHOW_GT = True
    SHOW_RANGE_LINES = True
    SHOW_RANGE_CIRCLES = False
    NUM_TIMESTEPS_KEEP_RANGES = 10

    # get the directory of the current file
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # get all the pyfg files inside the directory
    pyfg_files = _get_pyfg_files_in_dir(dir_path)
    pyfg_fnames = [os.path.basename(f) for f in pyfg_files]

    keep_looping = True
    loop_options = ["Visualize a data set", "Visualize measurement errors", "Exit"]

    # read the pyfg files
    while keep_looping:
        loop_choice = _select_from_list_based_on_requested_input(
            loop_options, "What would you like to do?"
        )
        if loop_choice == "Exit":
            keep_looping = False
            continue
        elif loop_choice == "Visualize a data set":
            pyfg_fname = _select_from_list_based_on_requested_input(
                pyfg_fnames, "Select a data set to visualize:"
            )
            if "mrclam" in pyfg_fname:
                print(
                    "\nWARNING:\n\nthe visualization isn't configured to handle the time-sync for the MRCLAM data set.\n"
                    "For example, the range measurements may not be in sync with the odometry.\n"
                    "This is only a visualization issue, the factor graph is still correct.\n\n"
                )
            pyfg_file = pyfg_files[pyfg_fnames.index(pyfg_fname)]
            fg = read_from_pyfg_text(pyfg_file)
            _visualize_dataset(
                fg,
                SHOW_GT,
                SHOW_RANGE_LINES,
                SHOW_RANGE_CIRCLES,
                NUM_TIMESTEPS_KEEP_RANGES,
            )
        elif loop_choice == "Visualize measurement errors":
            pyfg_fname = _select_from_list_based_on_requested_input(
                pyfg_fnames, "Select a data set to visualize:"
            )
            pyfg_file = pyfg_files[pyfg_fnames.index(pyfg_fname)]
            fg = read_from_pyfg_text(pyfg_file)
            _visualize_range_errors(fg)
            # _visualize_relative_pose_errors(fg)
        else:
            raise ValueError(f"Invalid loop choice. Received {loop_choice}.")
