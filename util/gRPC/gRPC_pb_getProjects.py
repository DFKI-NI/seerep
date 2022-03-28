#!/usr/bin/env python3

import sys

import grpc
import meta_operations_pb2_grpc as metaOperations
from google.protobuf import empty_pb2

server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = metaOperations.MetaOperationsStub(channel)

response = stub.GetProjects(empty_pb2.Empty())


print(server + " has the following projects (name/uuid):")
for projectinfo in response.projects:
    print("\t" + projectinfo.name + " " + projectinfo.uuid)
