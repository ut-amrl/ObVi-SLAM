# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import symforce

symforce.set_symbolic_api("symengine")
symforce.set_log_level("warning")

symforce.set_epsilon_to_symbol()

from symforce import codegen
from symforce.codegen import codegen_util
from symforce import ops
import symforce.symbolic as sf
from symforce.values import Values
from symforce.notebook_util import display, print_expression_tree, display_code, display_code_file


#
# from symforce.ops import StorageOps, GroupOps, LieGroupOps


def reprojectionResidual(
        robot_ax_ang: sf.Vector3, robot_transl: sf.Vector3, landmark: sf.Vector3, extrinsics: sf.Pose3, obs: sf.Vector2,
        pixel_err_mult_x: sf.Scalar, pixel_err_mult_y: sf.Scalar, epsilon: sf.Scalar = 0
) -> sf.Vector2:
    ax_ang_norm = robot_ax_ang.norm()
    rot_axis = robot_ax_ang / ax_ang_norm
    robot_rot_mat = sf.Rot3.from_angle_axis(angle=ax_ang_norm, axis=rot_axis)
    robot_pose = sf.Pose3(R=robot_rot_mat, t=robot_transl)

    world_pose_in_cam = robot_pose * extrinsics
    rel_pixel_position = world_pose_in_cam.inverse() * landmark

    camera_cal = sf.LinearCameraCal(
        focal_length=(1, 1),
        principal_point=(0, 0)
    )
    camera_point_reprojected, _ = camera_cal.pixel_from_camera_point(rel_pixel_position)

    error_mult_mat = sf.Matrix([[pixel_err_mult_x, 0], [0, pixel_err_mult_y]])
    return error_mult_mat * (camera_point_reprojected - obs)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    ax_ang = sf.Vector3.symbolic("r")
    robot_transl_sym = sf.Vector3.symbolic("t")
    landmark_sym =  sf.Vector3.symbolic("l")

    extrinsics_sym = sf.Pose3.symbolic("ext")
    obs_sym = sf.Vector2.symbolic("p")
    pixel_err_mult_x_sym = sf.Symbol("x_m")
    pixel_err_mult_y_sym = sf.Symbol("y_m")


    residual = reprojectionResidual(
            robot_ax_ang=ax_ang, robot_transl=robot_transl_sym, landmark=landmark_sym, extrinsics=extrinsics_sym,
            obs=obs_sym,
            pixel_err_mult_x=pixel_err_mult_x_sym, pixel_err_mult_y=pixel_err_mult_y_sym)

    display(residual)

    reprojetion_codegen = codegen.Codegen.function(
        func=reprojectionResidual,
        config=codegen.CppConfig()
    )
    reprojection_res_data = reprojetion_codegen.generate_function()

    print("Files generated in {}:\n".format(reprojection_res_data.output_dir))
    for f in reprojection_res_data.generated_files:
        print("  |- {}".format(f.relative_to(reprojection_res_data.output_dir)))

    display_code_file(reprojection_res_data.generated_files[0], "C++")

    codegen_with_jacobians = reprojetion_codegen.with_jacobians(
        # Just compute wrt the pose and point, not epsilon
        which_args=["robot_ax_ang", "robot_transl", "landmark"],
        # Include value, not just jacobians
        include_results=True,
    )

    data = codegen_with_jacobians.generate_function()
    print("Files generated in {}:\n".format(data.output_dir))
    for f in data.generated_files:
        print("  |- {}".format(f.relative_to(data.output_dir)))

    display_code_file(data.generated_files[0], "C++")
