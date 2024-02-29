#!/usr/bin/env python3

import flatbuffers
from grpc import Channel
from seerep.fb import ProjectInfo
from seerep.util.common import get_gRPC_channel
from seerep.util.fb_helper import createProjectRaw


def create_geo_proj(
    grpc_channel: Channel = get_gRPC_channel(),
) -> ProjectInfo.ProjectInfo:
    builder = flatbuffers.Builder(1024)

    response = createProjectRaw(
        grpc_channel,
        builder,
        "geodeticProject",
        "2",
        "EPSG::4326",
        4,
        6,
        7,
    )

    return response


if __name__ == "__main__":
    create_geo_proj()
