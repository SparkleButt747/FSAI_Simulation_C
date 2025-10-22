#pragma once

// Canonical cross-team data contracts for FS-AI components. These structs are
// shared across simulation, vision, and control modules and intentionally use C
// linkage so they can be consumed from pure C, C++, or generated code.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum FsaiPixelFormat {
    FSAI_PIXEL_RGB888 = 0,
    FSAI_PIXEL_NV12 = 1
} FsaiPixelFormat;

typedef struct FsaiCameraIntrinsics {
    float fx, fy;
    float cx, cy;
    float k1, k2, p1, p2, k3;
} FsaiCameraIntrinsics;

typedef struct FsaiCameraExtrinsics {
    float R[9];  // Row-major rotation body <- camera
    float t[3];  // Translation body <- camera (meters)
} FsaiCameraExtrinsics;

typedef struct FsaiFrame {
    uint64_t t_ns;     // Sensor timestamp (ns)
    int w, h;          // Resolution
    int stride;        // Bytes between rows
    FsaiPixelFormat fmt;
    FsaiCameraIntrinsics K;
    FsaiCameraExtrinsics T_bw;  // Body <- camera extrinsics
    uint8_t* data;               // Producer-owned buffer
} FsaiFrame;

typedef struct FsaiStereoFrame {
    FsaiFrame left;
    FsaiFrame right;
    uint64_t t_sync_ns;  // Mid-exposure stereo timestamp
} FsaiStereoFrame;

typedef enum FsaiConeSide {
    FSAI_CONE_LEFT = 0,
    FSAI_CONE_RIGHT = 1,
    FSAI_CONE_UNKNOWN = 2
} FsaiConeSide;

typedef struct FsaiConeDet {
    float p_B[3];  // Body-frame (x,y,z) meters
    FsaiConeSide side;
    float conf;    // [0,1]
    float cov[6];  // Optional covariance (xx,xy,xz,yy,yz,zz); zero if unused
} FsaiConeDet;

typedef struct FsaiDetections {
    uint64_t t_ns;     // Sensor time of the associated stereo frame
    int n;             // Number of detections (<=512)
    FsaiConeDet dets[512];
} FsaiDetections;

typedef struct FsaiVehicleState {
    uint64_t t_ns;
    float x, y;
    float yaw;
    float vx, vy;
    float yaw_rate;
    float ax, ay;
    float steer_rad;
} FsaiVehicleState;

typedef struct FsaiControlCmd {
    float steer_rad;
    float throttle;
    float brake;
    uint64_t t_ns;
} FsaiControlCmd;

typedef struct FsaiCanMsg {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint64_t t_ns;
} FsaiCanMsg;

#ifndef __cplusplus
typedef FsaiPixelFormat PixelFormat;
typedef FsaiCameraIntrinsics CameraIntrinsics;
typedef FsaiCameraExtrinsics CameraExtrinsics;
typedef FsaiFrame Frame;
typedef FsaiStereoFrame StereoFrame;
typedef FsaiConeSide ConeSide;
typedef FsaiConeDet ConeDet;
typedef FsaiDetections Detections;
typedef FsaiVehicleState VehicleState;
typedef FsaiControlCmd ControlCmd;
typedef FsaiCanMsg CanMsg;
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace fsai {
namespace types {
using PixelFormat = ::FsaiPixelFormat;
using CameraIntrinsics = ::FsaiCameraIntrinsics;
using CameraExtrinsics = ::FsaiCameraExtrinsics;
using Frame = ::FsaiFrame;
using StereoFrame = ::FsaiStereoFrame;
using ConeSide = ::FsaiConeSide;
using ConeDet = ::FsaiConeDet;
using Detections = ::FsaiDetections;
using VehicleState = ::FsaiVehicleState;
using ControlCmd = ::FsaiControlCmd;
using CanMsg = ::FsaiCanMsg;
}  // namespace types
}  // namespace fsai
#endif

