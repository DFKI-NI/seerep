#!/usr/bin/env python3
NUM_BB_LABELS = 5

import sys
import uuid
from typing import List

import flatbuffers
from grpc import Channel
from seerep.fb import DatasetUuidLabel, PointCloud2
from seerep.fb import point_cloud_service_grpc_fb as pointcloudService
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import (
    create_dataset_uuid_label,
    create_label,
    create_label_category,
    createQuery,
    getProject,
)


def add_pc_label_raw(target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()) -> List[bytearray]:
    builder = flatbuffers.Builder(1024)

    if target_proj_uuid is None:
        target_proj_uuid = getProject(builder, grpc_channel, "testproject")

        if target_proj_uuid is None:
            print("Requested project does not exist")
            sys.exit()

    stub = pointcloudService.PointCloudServiceStub(grpc_channel)

    query = createQuery(builder, projectUuids=[builder.CreateString(target_proj_uuid)], withoutData=True)
    builder.Finish(query)
    buf = builder.Output()

    msgToSend = []

    for responseBuf in stub.GetPointCloud2(bytes(buf)):
        response = PointCloud2.PointCloud2.GetRootAs(responseBuf)

        img_uuid = response.Header().UuidMsgs().decode("utf-8")
        projectUuid = response.Header().UuidProject().decode("utf-8")

        labelStr = ["label1", "label2"]
        labels = []

        for labelAct in labelStr:
            labels.append(
                create_label(
                    builder=builder, label=labelAct, label_id=1, instance_uuid=str(uuid.uuid4()), instance_id=2
                )
            )
        labelsCategory = []
        labelsCategory.append(
            create_label_category(
                builder=builder, labels=labels, datumaro_json="a very valid datumaro json", category="category P"
            )
        )

        dataset_uuid_label = create_dataset_uuid_label(
            builder=builder, projectUuid=projectUuid, datasetUuid=img_uuid, labels=labelsCategory
        )

        builder.Finish(dataset_uuid_label)
        buf = builder.Output()

        msgToSend.append(bytes(buf))

    response = stub.AddLabels(iter(msgToSend))
    return msgToSend


def add_pc_label(
    target_proj_uuid: str = None, grpc_channel: Channel = get_gRPC_channel()
) -> List[DatasetUuidLabel.DatasetUuidLabel]:
    return [
        DatasetUuidLabel.DatasetUuidLabel.GetRootAs(resp_buf)
        for resp_buf in add_pc_label_raw(target_proj_uuid, grpc_channel)
    ]


if __name__ == "__main__":
    label_list: List[DatasetUuidLabel.DatasetUuidLabel] = add_pc_label()

    for label in label_list:
        for labelCategory_idx in range(label.LabelsLength()):
            for label_idx in range(label.Labels(labelCategory_idx).LabelsLength()):
                print(f"uuid: {label.Labels(labelCategory_idx).Labels(label_idx).Label().decode()}")
