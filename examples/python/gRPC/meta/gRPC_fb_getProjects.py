#!/usr/bin/env python3
from typing import List

import flatbuffers
from grpc import Channel
from seerep.fb import Empty, ProjectInfos
from seerep.fb import meta_operations_grpc_fb as metaOperations
from seerep.util.common import get_gRPC_channel


def get_projects(grpc_channel: Channel = get_gRPC_channel()) -> List:
    stub = metaOperations.MetaOperationsStub(grpc_channel)

    builder = flatbuffers.Builder(1024)
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stub.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    projects_list: List = []

    print("The server has the following projects (name/uuid):")
    for i in range(response.ProjectsLength()):
        print(
            "\t"
            + response.Projects(i).Name().decode("utf-8")
            + " "
            + response.Projects(i).Uuid().decode("utf-8")
        )
        projects_list.append(response.Projects(i))

    return projects_list


if __name__ == "__main__":
    get_projects()
