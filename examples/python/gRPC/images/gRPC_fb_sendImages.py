#!/usr/bin/env python3
import random
import time
import uuid
from typing import Iterator, List, Tuple, Union
from uuid import uuid4

import flatbuffers
import numpy as np
from grpc import Channel
from quaternion import quaternion
from seerep.fb import ServerResponse
from seerep.fb import (
    camera_intrinsics_service_grpc_fb as camera_intrinsic_service,
)
from seerep.fb import image_service_grpc_fb as image_service
from seerep.fb import tf_service_grpc_fb as tf_service
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_label,
    create_label_category,
    createCameraIntrinsics,
    createHeader,
    createImage,
    createProject,
    createQuaternion,
    createRegionOfInterest,
    createTimeStamp,
    createTransform,
    createTransformStamped,
    createVector3,
)


def send_images(
    grpc_channel: Channel,
    project_uuid: str,
    camera_intrinsic_uuid: str,
    image_payloads: List[Union[np.ndarray, str]],
    timestamps: Union[List[Tuple[int, int]], None] = None,
    frame_id: str = "camera",
) -> List[bytes]:
    """
    Send images to a SEEREP project

    Args:
        grpc_channel (Channel): The gRPC channel.
        project_uuid (str): The UUID of the project.
        camera_intrinsic_uuid (str): The UUID of the corresponding camera
        intrinsics.
        image_payloads (List[Union[np.ndarray, str]]): A list of image payloads.
        Each payload can be either a numpy array (actual data) or a string
        (for a remote stroage URI).
        timestamps (Union[List[Tuple[int, int]], None]): A list of timestamps
        to attach to the images. Has to be the same size as image_payloads.

    Returns:
        List[bytes]: A list of bytes representing the serialized
        FlatBuffers messages.

    """
    if timestamps and len(timestamps) != len(image_payloads):
        print(
            "Couldn't send images, "
            "due to len(image_payloads) != len(timestamps)"
        )
        return []

    fbb = flatbuffers.Builder()
    image_service_stub = image_service.ImageServiceStub(grpc_channel)

    # Use current time as timestamp
    timestamp = createTimeStamp(
        fbb, *list(map(int, str(time.time()).split(".")))
    )

    labels = [
        create_label(
            builder=fbb,
            label=label_str,
            label_id=i,
            instance_uuid=str(uuid4()),
            instance_id=i + 100,
        )
        for i, label_str in enumerate(["label1", "label2"])
    ]

    labelsCategory = [
        create_label_category(
            builder=fbb,
            labels=labels,
            datumaro_json="a very valid datumaro json",
            category="category A",
        )
    ]

    fb_msgs = []
    for idx, image in enumerate(image_payloads):
        if timestamps is None:
            header = createHeader(
                fbb, timestamp, frame_id, project_uuid, str(uuid4())
            )
        else:
            timestamp = createTimeStamp(fbb, *timestamps[idx])
            header = createHeader(
                fbb, timestamp, frame_id, project_uuid, str(uuid4())
            )

        fb_image = createImage(
            fbb,
            header,
            "rgb8",
            True,
            camera_intrinsic_uuid,
            height=image.shape[0] if type(image) is np.ndarray else 480,
            width=image.shape[1] if type(image) is np.ndarray else 640,
            image=image if type(image) is np.ndarray else None,
            uri=None if type(image) is np.ndarray else image,
            labels=labelsCategory,
        )
        fbb.Finish(fb_image)
        fb_msgs.append(bytes(fbb.Output()))

    image_service_stub.TransferImage(iter(fb_msgs))
    return fb_msgs


def generate_image_ressources(num_images: int) -> List[Union[np.ndarray, str]]:
    """
    Generate an image ressource which can either be a URI or random image data.

    Args:
        num_images (int): The number of payloads to generate.

    Returns:
        List[Union[np.ndarray, str]]: A list of image ressources.
    """
    ressources = []
    for i in range(num_images):
        if random.choice([True, False]):
            ressources.append(f"https://example.com/image{i}.jpg")
        else:
            ressources.append(np.random.rand(640, 480, 3).astype(np.uint8))
    return ressources


def create_project(grpc_channel: Channel, project_name: str) -> str:
    """
    Creates a new project in SEEREP.

    Args:
        grpc_channel (Channel): The gRPC channel to communicate with the server.
        project_name (str): The name of the project to create.

    Returns:
        str: The UUID of the created project.
    """
    fbb = flatbuffers.Builder()
    return createProject(
        grpc_channel, fbb, project_name, "map", "WGS84", 0.0, 85.0, 23.0
    )


def send_cameraintrinsics(
    grpc_channel: Channel, project_uuid: str, frame_id: str = "map"
) -> str:
    """
    Create and send a camera intrinsics object to SEEREP.

    Args:
        grpc_channel (Channel): The gRPC channel to communicate with the server.
        project_uuid (str): The UUID of the project.

    Returns:
        str: The UUID of the created camera intrinsics.
    """
    fbb = flatbuffers.Builder()
    camera_intrinsic_service_stub = (
        camera_intrinsic_service.CameraIntrinsicsServiceStub(grpc_channel)
    )
    timestamp = createTimeStamp(fbb, 0, 0)
    ci_uuid = str(uuid4())
    header = createHeader(fbb, timestamp, frame_id, project_uuid, ci_uuid)
    roi = createRegionOfInterest(fbb, 0, 0, 0, 0, False)

    distortion_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    rect_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    intrins_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12]
    proj_matrix = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    cameraintrinsics = createCameraIntrinsics(
        fbb,
        header,
        3,
        4,
        "plumb_bob",
        distortion_matrix,
        intrins_matrix,
        rect_matrix,
        proj_matrix,
        4,
        5,
        roi,
        5,
    )
    fbb.Finish(cameraintrinsics)
    camera_intrinsic_service_stub.TransferCameraIntrinsics(bytes(fbb.Output()))
    return ci_uuid


def send_tfs(
    grpc_channel: Channel,
    project_uuid: str,
    timestamps: List[Tuple[int, int]],
    frame_id: str = "map",
    child_frame_id: str = "camera",
) -> bytes:
    """
    Stream len(timestamps) transforms with the form frame_id -> child_frame_id
    to SEEREP.

    Args:
        grpc_channel (Channel): The grpc_channel to a SEEREP server.
        project_uuid (str): The project target for the transformations.
        timestamps (List[Tuple[int, int]]): The timestamps of
        the transformation.
        frame_id (str): The parent frame for the transformations.
        child_frame_id (str): The child frame for the transformations.
    """

    def tfs_gen() -> Iterator[bytes]:
        builder = flatbuffers.Builder(1024)
        quat = createQuaternion(builder, quaternion(1, 0, 0, 0))

        for idx, (seconds, nanos) in enumerate(timestamps):
            timestamp = createTimeStamp(builder, seconds, nanos)

            header = createHeader(
                builder, timestamp, frame_id, project_uuid, str(uuid.uuid4())
            )

            translation = createVector3(
                builder,
                np.array(
                    [100 * idx**2, 100 * idx**2, 30 * idx], dtype=np.float64
                ),
            )

            tf = createTransform(builder, translation, quat)
            tfs = createTransformStamped(builder, child_frame_id, header, tf)

            builder.Finish(tfs)
            yield bytes(builder.Output())

    return tf_service.TfServiceStub(grpc_channel).TransferTransformStamped(
        tfs_gen()
    )


if __name__ == "__main__":
    channel = get_gRPC_channel()
    project_uuid = create_project(channel, "testproject")
    camera_uuid = send_cameraintrinsics(channel, project_uuid)

    timestamp_nanos = 1245
    nanos_factor = 1e-9

    timestamps = [
        (t, timestamp_nanos) for t in range(1661336507, 1661336606, 10)
    ]

    img_bufs = send_images(
        channel,
        project_uuid,
        camera_uuid,
        generate_image_ressources(10),
        timestamps,
    )

    tf_resp = send_tfs(channel, project_uuid, timestamps)

    if img_bufs is not None and len(img_bufs) > 0:
        print("Images sent successfully")

    if (
        ServerResponse.ServerResponse.GetRootAs(tf_resp).Message().decode()
        == "everything stored!"
    ):
        print("Transforms sent successfully")
