# test file for
#   gRPC_fb_queryPointCloud.py
#   gRPC_fb_sendPointCloud.py
from typing import List, Sequence, Tuple, Union
from uuid import uuid4

import flatbuffers
import numpy as np
from grpc import Channel
from quaternion import quaternion as create_quat
from seerep.fb import (
    PointCloud2,
)
from seerep.fb import point_cloud_service_grpc_fb as pointCloudService
from seerep.fb import tf_service_grpc_fb as tf_service
from seerep.util.fb_helper import (
    addToPointFieldVector,
    create_label,
    create_label_category,
    createHeader,
    createPoint2d,
    createPointFields,
    createPolygon2D,
    createQuaternion,
    createQuery,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createVector3,
)
from seerep.util.fb_to_dict import SchemaFileNames, fb_flatc_dict


class Point3:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


def create_point_cloud_aabb(
    builder,
    header,
    aabb_point1: Point3,
    aabb_point2: Point3,
    num_labels: int = 1,
):
    """Creates a flatbuffers point cloud message in form of a AABB
    using min_point and max_point"""
    pointFields = createPointFields(builder, ["x", "y", "z", "rgba"], 7, 4, 1)
    pointFieldsVector = addToPointFieldVector(builder, pointFields)

    # create labels
    labelsStrings = [f"Label{i}" for i in range(num_labels)]
    instance_uuids = [str(uuid4()) for _ in range(num_labels)]

    labels = []
    for i in range(len(labelsStrings)):
        labels.append(
            create_label(
                builder=builder,
                label=labelsStrings[i],
                label_id=1,
                instance_uuid=instance_uuids[i],
                instance_id=5,
            )
        )
    labelsCategory = []
    labelsCategory.append(
        create_label_category(
            builder=builder,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category Z",
        )
    )

    PointCloud2.StartLabelsVector(builder, len(labelsCategory))
    for label in labelsCategory:
        builder.PrependUOffsetTRelative(label)
    labelsCatOffset = builder.EndVector()

    # Note: rgb field is float, for simplification
    pointsBox = [[]]
    x_min = min(aabb_point1.x, aabb_point2.x)
    y_min = min(aabb_point1.y, aabb_point2.y)
    z_min = min(aabb_point1.z, aabb_point2.z)
    x_max = max(aabb_point1.x, aabb_point2.x)
    y_max = max(aabb_point1.y, aabb_point2.y)
    z_max = max(aabb_point1.y, aabb_point2.y)
    # for x in range(int(x_min), int(x_max) + 1, 1):
    #     for y in range(int(y_min), int(y_max) + 1, 1):
    #         for z in range(int(z_min), int(z_max) + 1, 1):
    #             pointsBox[0].append([x, y, z, 0])
    # also include the minimal and maximal bounding point
    pointsBox[0].append([x_min, y_min, z_min, 0])
    pointsBox[0].append([x_max, y_max, z_max, 0])
    points = np.array(pointsBox).astype(np.float32)

    pointsVector = builder.CreateByteVector(points.tobytes())

    # add all data into the flatbuffers point cloud message
    PointCloud2.Start(builder)
    PointCloud2.AddHeader(builder, header)
    PointCloud2.AddHeight(builder, points.shape[0])
    PointCloud2.AddWidth(builder, points.shape[1])
    PointCloud2.AddIsBigendian(builder, True)
    PointCloud2.AddPointStep(builder, 16)
    PointCloud2.AddRowStep(builder, points.shape[1] * 16)
    PointCloud2.AddFields(builder, pointFieldsVector)
    PointCloud2.AddData(builder, pointsVector)
    PointCloud2.AddLabels(builder, labelsCatOffset)
    return PointCloud2.End(builder)


def send_pcs(
    grpc_channel: Channel,
    pcs: Union[int, Sequence[int], bytearray, Sequence[bytearray]],
) -> List[bytearray]:
    """sends a serialized pointcloud flatbuffers message"""
    if (
        type(pcs) is not int
        and type(pcs) is not bytes
        and (type(pcs) is not bytearray)
    ):
        if len(pcs) > 0 and type(pcs[0]) is int:
            pcs = [bytes(pc) for pc in pcs]
    elif type(pcs) is int or type(pcs) is bytearray:
        pcs = [bytes(pcs)]
    else:
        pcs = [pcs]

    stub = pointCloudService.PointCloudServiceStub(grpc_channel)
    stub.TransferPointCloud2(iter(pcs))
    return pcs


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


def test_queryPCPreciseNoIntersect(grpc_channel, project_setup):
    _, proj_uuid = project_setup

    builder = flatbuffers.Builder()
    pcl_stub = pointCloudService.PointCloudServiceStub(grpc_channel)

    timestamp_secs = 1661336507
    timestamp_nanos = 1245

    ts_obj = createTimeStamp(builder, timestamp_secs, timestamp_nanos)

    header = createHeader(builder, ts_obj, "pc_test", proj_uuid, str(uuid4()))

    pc_msg = create_point_cloud_aabb(
        builder, header, Point3(0, 0, 0), Point3(3, 2, 2)
    )
    builder.Finish(pc_msg)
    sent_pcs = send_pcs(grpc_channel, builder.Output())

    translation = (-4, -4, -2)
    # This is equivalent to XYZ Euler:
    #   X = 45°
    #   Y = 0°
    #   Z = 0°
    quaternion = (0.924, 0.383, 0, 0)
    send_tf(
        grpc_channel,
        proj_uuid,
        (timestamp_secs, timestamp_nanos),
        translation,
        quaternion,
        child_frame_id="pc_test",
    )

    builder = flatbuffers.Builder()
    # build the query
    verts = [
        createPoint2d(builder, *p)
        for p in [(-4, -3), (-2, -3), (-2, -1), (-4, -1)]
    ]
    polygon = createPolygon2D(builder, height=2, z=0.5, vertices=verts)
    query = createQuery(builder, polygon2d=polygon)
    builder.Finish(query)
    queried_pcs = pcl_stub.GetPointCloud2(bytes(builder.Output()))

    # convert the sent/queried pcs
    sent_pcs = sorted(
        [fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2) for pcl in sent_pcs],
        key=lambda pc: pc["header"]["uuid_msgs"],
    )

    queried_pcs = sorted(
        [
            fb_flatc_dict(pcl, SchemaFileNames.POINT_CLOUD_2)
            for pcl in queried_pcs
        ],
        key=lambda pc: pc["header"]["uuid_msgs"],
    )

    assert len(queried_pcs) == 0
    assert len(sent_pcs) == 1
