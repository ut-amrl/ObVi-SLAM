//
// Created by amanda on 7/17/23.
//

#ifndef UT_VSLAM_CAMERA_INFO_IO_UTILS_H
#define UT_VSLAM_CAMERA_INFO_IO_UTILS_H

#include <file_io/camera_extrinsics_with_id_io.h>
#include <file_io/camera_intrinsics_with_id_io.h>
#include <refactoring/types/vslam_basic_types_refactor.h>

namespace file_io {

std::unordered_map<vslam_types_refactor::CameraId,
                   vslam_types_refactor::CameraIntrinsicsMat<double>>
readCameraIntrinsicsByCameraFromFile(const std::string &file_name) {
  std::vector<file_io::CameraIntrinsicsWithId> camera_intrinsics_by_cam_id;
  file_io::readCameraIntrinsicsWithIdsFromFile(file_name,
                                               camera_intrinsics_by_cam_id);
  std::unordered_map<vslam_types_refactor::CameraId,
                     vslam_types_refactor::CameraIntrinsicsMat<double>>
      cam_intrinsics_map;
  for (const file_io::CameraIntrinsicsWithId &intrinsics_for_cam :
       camera_intrinsics_by_cam_id) {
    vslam_types_refactor::CameraIntrinsicsMat<double> intrinsics_mat;
    intrinsics_mat << intrinsics_for_cam.mat_00, intrinsics_for_cam.mat_01,
        intrinsics_for_cam.mat_02, intrinsics_for_cam.mat_10,
        intrinsics_for_cam.mat_11, intrinsics_for_cam.mat_12,
        intrinsics_for_cam.mat_20, intrinsics_for_cam.mat_21,
        intrinsics_for_cam.mat_22;
    cam_intrinsics_map[intrinsics_for_cam.camera_id] = intrinsics_mat;
    // TODO do we need to include the width/height anywhere?
  }
  return cam_intrinsics_map;
}

std::unordered_map<vslam_types_refactor::CameraId,
                   vslam_types_refactor::CameraExtrinsics<double>>
readCameraExtrinsicsByCameraFromFile(const std::string &file_name) {
  std::unordered_map<vslam_types_refactor::CameraId,
                     vslam_types_refactor::CameraExtrinsics<double>>
      extrinsics_map;

  // [0 -1 0; 0 0 -1; 1 0 0] is the rotation of the camera matrix from a classic
  // world frame - for the camera +z is the +x world axis, +y is the -z world
  // axis, and +x is the -y world axis
  // TODO Verify that this is correct (and eventually just update the file to
  // have this)
  //  Eigen::Quaterniond extrinsics_orientation_switch_to_cam =
  //      Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5)
  //          .inverse();  // [0 -1 0; 0 0 -1; 1 0 0]^-1

  std::vector<file_io::CameraExtrinsicsWithId> camera_extrinsics_by_cam_id;
  file_io::readCameraExtrinsicsWithIdsFromFile(file_name,
                                               camera_extrinsics_by_cam_id);
  for (const file_io::CameraExtrinsicsWithId &extrinsics_for_cam :
       camera_extrinsics_by_cam_id) {
    vslam_types_refactor::Position3d<double> extrinsics_pos(
        extrinsics_for_cam.transl_x,
        extrinsics_for_cam.transl_y,
        extrinsics_for_cam.transl_z);

    //    vslam_types_refactor::Orientation3D<double> extrinsics_orient(
    //        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
    //                           extrinsics_for_cam.quat_x,
    //                           extrinsics_for_cam.quat_y,
    //                           extrinsics_for_cam.quat_z) *
    //        extrinsics_orientation_switch_to_cam);
    vslam_types_refactor::Orientation3D<double> extrinsics_orient(
        Eigen::Quaterniond(extrinsics_for_cam.quat_w,
                           extrinsics_for_cam.quat_x,
                           extrinsics_for_cam.quat_y,
                           extrinsics_for_cam.quat_z));
    vslam_types_refactor::CameraExtrinsics<double> extrinsics_obj(
        extrinsics_pos, extrinsics_orient);

    extrinsics_map[extrinsics_for_cam.camera_id] = extrinsics_obj;
  }
  return extrinsics_map;
}
}  // namespace file_io

#endif  // UT_VSLAM_CAMERA_INFO_IO_UTILS_H
