#!/usr/bin/env python3

import sys

import frame_query_pb2 as frameQuery
import grpc
import meta_operations_pb2_grpc as metaOperations
import tf_service_pb2_grpc as tfService
from google.protobuf import empty_pb2

# import numpy as np


channel = grpc.insecure_channel("localhost:9090")

stub = tfService.TfServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

response = stubMeta.GetProjects(empty_pb2.Empty())

projectuuid = ""
for project in response.projects:
    print(project.name + " " + project.uuid)
    if project.name == "testproject":
        projectuuid = project.uuid

if projectuuid == "":
    sys.exit()


theQuery = frameQuery.FrameQuery()
theQuery.projectuuid = projectuuid


frames = stub.GetFrames(theQuery)

for frame in frames.frames:
    print("frame: " + frame)
