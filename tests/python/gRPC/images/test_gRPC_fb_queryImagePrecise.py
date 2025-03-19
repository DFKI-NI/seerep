# test file for
#   gRPC_fb_sendImages.py
#   gRPC_fb_queryImages.py

from typing import Tuple
from uuid import uuid4

import flatbuffers
from grpc import Channel
from gRPC.images import gRPC_fb_queryImages, gRPC_fb_sendImages
from quaternion import quaternion as create_quat
from seerep.fb import (
    camera_intrinsics_service_grpc_fb as camera_intrinsic_service,
)
from seerep.fb import tf_service_grpc_fb as tf_service
from seerep.util.fb_helper import (
    createCameraIntrinsics,
    createHeader,
    createPoint2d,
    createPolygon2D,
    createQuaternion,
    createRegionOfInterest,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createVector3,
)
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


def send_simple_camintrinsics(
    grpc_channel: Channel, project_uuid: str, frame_id: str = "camera"
) -> str:
    """
    Create a simple Camera intrinsics with the Frustum in the form of
    a square pyramid with a side length and a height of 1.

    Returns:
        str: The uuid of the created camera intrinsics.
    """
    fbb = flatbuffers.Builder()
    camera_intrinsic_service_stub = (
        camera_intrinsic_service.CameraIntrinsicsServiceStub(grpc_channel)
    )
    timestamp = createTimeStamp(fbb, 0, 0)
    ci_uuid = str(uuid4())
    header = createHeader(fbb, timestamp, frame_id, project_uuid, ci_uuid)
    roi = createRegionOfInterest(fbb, 0, 0, 0, 0, False)

    distortion_matrix: list[float] = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    rect_matrix: list[float] = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    # important for frustum calculations are the indices 0 and 4
    # the frustum calculation can be found in
    # seerep_hdf5/seerep_hdf5_core/src/hdf5_core_image.cpp
    intrins_matrix: list[float] = [1, 5, 6, 7, 1, 9, 10, 11, 12]
    proj_matrix: list[float] = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    ci_height = 1
    ci_width = 1

    max_view_dist = 1

    cameraintrinsics = createCameraIntrinsics(
        fbb,
        header,
        ci_height,
        ci_width,
        "plumb_bob",
        distortion_matrix,
        intrins_matrix,
        rect_matrix,
        proj_matrix,
        4,
        5,
        roi,
        max_view_dist,
    )
    fbb.Finish(cameraintrinsics)
    camera_intrinsic_service_stub.TransferCameraIntrinsics(bytes(fbb.Output()))
    return ci_uuid


def send_tf(
    grpc_channel: Channel,
    project_uuid: str,
    timestamp: Tuple[int, int],
    translation: Tuple[float, float, float],
    quaternion: Tuple[float, float, float, float],
    frame_id: str = "map",
    child_frame_id="camera",
) -> bytes:
    """
    send a frame_id -> child_frame_id transform.

    Args:
        grpc_channel (Channel): The grpc_channel to a SEEREP server.
        project_uuid (str): The project target for the transformations.
        timestamp (Tuple[int, int]): The timestamp for the sent tf.
        translation (Tuple[float, float, float]): the translation vector
            in the form (x, y, z)
        quaternion (int): the quaternion representing the orientation
            of the tf in (w, x, y, z)
        frame_id (str): The parent frame for the transformations.
        child_frame_id (str): The child frame for the transformations.

    Return:
        the serialized tf object
    """
    builder = flatbuffers.Builder(1024)

    secs, nanos = timestamp
    ts = createTimeStamp(builder, secs, nanos)

    header = createHeader(builder, ts, frame_id, project_uuid, str(uuid4()))

    trans = createVector3(builder, translation)
    quat = createQuaternion(builder, create_quat(*quaternion))

    tf = createTransform(builder, trans, quat)
    tfs = createTransformStamped(builder, child_frame_id, header, tf)

    builder.Finish(tfs)
    serialized_tf = bytes(builder.Output())
    tf_service.TfServiceStub(grpc_channel).TransferTransformStamped(
        iter([serialized_tf])
    )
    return serialized_tf


def test_queryImagePreciseIntersect(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    timestamp_secs = 1661336507
    timestamp_nanos = 1245

    ts = (timestamp_secs, timestamp_nanos)

    camera_intrinsics_uuid = send_simple_camintrinsics(grpc_channel, proj_uuid)

    sent_images = gRPC_fb_sendImages.send_images(
        grpc_channel,
        proj_uuid,
        camera_intrinsics_uuid,
        gRPC_fb_sendImages.generate_image_ressources(1),
        1 * [ts],
    )

    # make sure the frustum is above the query cube
    trans = (-0.5, -0.5, 1.5)

    # this is equivalent to a Euler XYZ rotation with:
    #   X = 225°
    #   Y = 0°
    #   Z = -45°
    rot = (-0.354, 0.854, -0.354, 0.146)
    send_tf(grpc_channel, proj_uuid, ts, trans, rot)

    builder = flatbuffers.Builder(1024)

    verts = [
        createPoint2d(builder, *p) for p in [(0, 0), (1, 0), (1, 1), (0, 1)]
    ]

    polygon = createPolygon2D(builder, height=1, z=0, vertices=verts)

    queried_images = gRPC_fb_queryImages.query_images_raw(
        builder, grpc_channel, proj_uuid, polygon2d=polygon
    )

    sent_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in sent_images
    ]

    queried_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in queried_images
    ]

    assert sorted(
        sent_images, key=lambda img: img["header"]["uuid_msgs"]
    ) == sorted(queried_images, key=lambda img: img["header"]["uuid_msgs"])


# this test should detect no intersection with the precise method
# but would fail with the AABB approximation
def test_queryImagePreciseNoIntersect(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    timestamp_secs = 1661336507
    timestamp_nanos = 1245

    ts = (timestamp_secs, timestamp_nanos)

    camera_intrinsics_uuid = send_simple_camintrinsics(grpc_channel, proj_uuid)

    sent_images = gRPC_fb_sendImages.send_images(
        grpc_channel,
        proj_uuid,
        camera_intrinsics_uuid,
        gRPC_fb_sendImages.generate_image_ressources(1),
        1 * [ts],
    )

    trans = (-2.3, 2, 1)
    rot = (1, 0, 0, 0)
    send_tf(grpc_channel, proj_uuid, ts, trans, rot)

    builder = flatbuffers.Builder(1024)

    verts = [
        createPoint2d(builder, *p) for p in [(-1, 1), (-1, 3), (-2, 3), (-2, 1)]
    ]

    polygon = createPolygon2D(builder, height=2, z=-0.5, vertices=verts)

    queried_images = gRPC_fb_queryImages.query_images_raw(
        builder, grpc_channel, proj_uuid, polygon2d=polygon
    )

    sent_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in sent_images
    ]

    queried_images = [
        fb_flatc_dict(img, SchemaFileNames.IMAGE) for img in queried_images
    ]

    assert len(sent_images) > 0
    assert len(queried_images) == 0
