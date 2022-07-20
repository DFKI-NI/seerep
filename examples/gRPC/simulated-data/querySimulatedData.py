#!/usr/bin/env python3

import os
import sys

import flatbuffers
import grpc
from query_pb2 import Query
from seerep.fb import (
    Boundingbox,
    Empty,
    Header,
    Image,
    Point,
    ProjectInfos,
    Query,
    TimeInterval,
    Timestamp,
)
from seerep.fb import image_service_grpc_fb as imageService
from seerep.fb import meta_operations_grpc_fb as metaOperations

# import numpy as np


# # server with certs
# __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
# with open(os.path.join(__location__, 'tls.pem'), 'rb') as f:
#     root_cert = f.read()
# server = "seerep.robot.10.249.3.13.nip.io:32141"
# creds = grpc.ssl_channel_credentials(root_cert)
# channel = grpc.secure_channel(server, creds)

# server without certs
server = "localhost:9090"
channel = grpc.insecure_channel(server)

stub = imageService.ImageServiceStub(channel)
stubMeta = metaOperations.MetaOperationsStub(channel)

builder = flatbuffers.Builder(1024)
Empty.Start(builder)
emptyMsg = Empty.End(builder)
builder.Finish(emptyMsg)
buf = builder.Output()

responseBuf = stubMeta.GetProjects(bytes(buf))
response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

projectuuid = ""
for i in range(response.ProjectsLength()):
    print(response.Projects(i).Name().decode("utf-8") + " " + response.Projects(i).Uuid().decode("utf-8"))
    if response.Projects(i).Name().decode("utf-8") == "simulatedData":
        projectuuid = response.Projects(i).Uuid().decode("utf-8")

if projectuuid == "":
    sys.exit()

Point.Start(builder)
Point.AddX(builder, -5.0)
Point.AddY(builder, -5.0)
Point.AddZ(builder, -100.0)
pointMin = Point.End(builder)

Point.Start(builder)
Point.AddX(builder, 5.0)
Point.AddY(builder, 5.0)
Point.AddZ(builder, 100.0)
pointMax = Point.End(builder)

frameId = builder.CreateString("map")
Header.Start(builder)
Header.AddFrameId(builder, frameId)
header = Header.End(builder)

Boundingbox.Start(builder)
Boundingbox.AddPointMin(builder, pointMin)
Boundingbox.AddPointMax(builder, pointMax)
Boundingbox.AddHeader(builder, header)
boundingbox = Boundingbox.End(builder)

Timestamp.Start(builder)
Timestamp.AddSeconds(builder, 1654688920)
Timestamp.AddNanos(builder, 0)
timeMin = Timestamp.End(builder)

Timestamp.Start(builder)
Timestamp.AddSeconds(builder, 1654688940)
Timestamp.AddNanos(builder, 0)
timeMax = Timestamp.End(builder)

TimeInterval.Start(builder)
TimeInterval.AddTimeMin(builder, timeMin)
TimeInterval.AddTimeMax(builder, timeMax)
timeInterval = TimeInterval.End(builder)

projectuuidString = builder.CreateString(projectuuid)
Query.StartProjectuuidVector(builder, 1)
builder.PrependUOffsetTRelative(projectuuidString)
projectuuidMsg = builder.EndVector()


label = builder.CreateString("http://aims.fao.org/aos/agrovoc/c_14385")
Query.StartLabelVector(builder, 1)
builder.PrependUOffsetTRelative(label)
labelMsg = builder.EndVector()

Query.Start(builder)
Query.AddBoundingbox(builder, boundingbox)
Query.AddTimeinterval(builder, timeInterval)
Query.AddProjectuuid(builder, projectuuidMsg)
Query.AddLabel(builder, labelMsg)
queryMsg = Query.End(builder)

builder.Finish(queryMsg)
buf = builder.Output()

i = 0
for responseBuf in stub.GetImage(bytes(buf)):
    response = Image.Image.GetRootAs(responseBuf)
    print(str(i) + "uuidmsg: " + response.Header().UuidMsgs().decode("utf-8"))
    i = i + 1
    if response.LabelsBbLength() > 0:
        print("first label: " + response.LabelsBb(0).LabelWithInstance().Label().decode("utf-8"))
        print(
            "first BoundingBox (Xmin,Ymin,Xmax,Ymax): "
            + str(response.LabelsBb(0).BoundingBox().PointMin().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMin().Y())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().X())
            + " "
            + str(response.LabelsBb(0).BoundingBox().PointMax().Y())
        )
