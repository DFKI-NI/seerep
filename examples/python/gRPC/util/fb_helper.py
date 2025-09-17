import sys
from typing import Dict, List, Tuple, Union

import numpy as np
from flatbuffers import Builder
from grpc import Channel
from seerep.fb import (
    Boundingbox,
    CameraIntrinsics,
    CameraIntrinsicsQuery,
    DatasetUuidLabel,
    Empty,
    GeodeticCoordinates,
    Header,
    Image,
    Label,
    LabelCategory,
    Point,
    PointCloud2,
    PointField,
    PointStamped,
    Polygon2D,
    ProjectCreation,
    ProjectInfo,
    ProjectInfos,
    Quaternion,
    Query,
    QueryInstance,
    RegionOfInterest,
    TimeInterval,
    Timestamp,
    Transform,
    TransformStamped,
    TransformStampedIntervalQuery,
    TransformStampedQuery,
    UuidDatatypePair,
    UuidDatatypeWithCategory,
    Vector3,
)
from seerep.fb import meta_operations_grpc_fb as metaOperations


# ruff: noqa: PLR0911
# TODO: should be moved into a separate module
def rosToNumpyDtype(ros_dtype: int) -> np.dtype:
    """
    Converts the numeric representations of dtypes in ROS\
        to numpy dtype objects.

    Args:
        ros_dtype: The numeric representation of the dtype in ROS\
            (see [ROS documentation](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointField.html))

    Returns:
        The numpy dtype equivalent
    """
    if ros_dtype == 1:
        return np.dtype(np.int8)
    elif ros_dtype == 2:
        return np.dtype(np.uint8)
    elif ros_dtype == 3:
        return np.dtype(np.int16)
    elif ros_dtype == 4:
        return np.dtype(np.uint16)
    elif ros_dtype == 5:
        return np.dtype(np.int32)
    elif ros_dtype == 6:
        return np.dtype(np.uint32)
    elif ros_dtype == 7:
        return np.dtype(np.float32)
    elif ros_dtype == 8:
        return np.dtype(np.float64)
    else:
        raise ValueError("Unknown dtype")


def getProject(
    builder: Builder, channel: Channel, name: str
) -> Union[str, None]:
    """
    Retrieve a project by name.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project

    Returns:
        The UUID of the project if found, None otherwise
    """
    stubMeta = metaOperations.MetaOperationsStub(channel)

    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    for i in range(response.ProjectsLength()):
        if response.Projects(i).Name().decode("utf-8") == name:
            return response.Projects(i).Uuid().decode("utf-8")
    return None


def getProjectInfo(
    builder: Builder, channel: Channel, name: str
) -> Union[Dict[str, str], None]:
    """
    Retrieve project infos by name.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project

    Returns:
        A dictionary containing the project information or None
    """
    stubMeta = metaOperations.MetaOperationsStub(channel)

    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    buf = builder.Output()

    responseBuf = stubMeta.GetProjects(bytes(buf))
    response = ProjectInfos.ProjectInfos.GetRootAs(responseBuf)

    for i in range(response.ProjectsLength()):
        if response.Projects(i).Name().decode("utf-8") == name:
            project = response.Projects(i)
            return {
                "name": project.Name().decode("utf-8"),
                "uuid": project.Uuid().decode("utf-8"),
                "frameid": project.Frameid().decode("utf-8"),
                "geodetic_position": f"lat: \
                    {project.GeodeticPosition().Latitude()},\
                    long: {project.GeodeticPosition().Longitude()},\
                    alt: {project.GeodeticPosition().Altitude()}",
                "coordinate_system": project.GeodeticPosition()
                .CoordinateSystem()
                .decode("utf-8"),
            }

    return None


def createProjectRaw(
    channel: Channel,
    builder: Builder,
    name: str,
    frameId: str,
    coordSys: str,
    altitude: float,
    latitude: float,
    longitude: float,
) -> bytearray:
    """
    Create a project from the parameters and return it as a flatbuffers object.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        name: The name of the project
        frameId: The coordinate frame of the project
        coordSys: The coordinate system type as a
        [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        latitude: The latitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        longitude: The longitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)

    Returns:
        A flatbuffers object of type ProjectInfo representing the project
    """
    stubMeta = metaOperations.MetaOperationsStub(channel)

    frameIdBuf = builder.CreateString(frameId)
    nameBuf = builder.CreateString(name)

    # create a geodetic coordinates object
    coordSysBuf = builder.CreateString(coordSys)

    GeodeticCoordinates.Start(builder)
    GeodeticCoordinates.AddCoordinateSystem(builder, coordSysBuf)
    GeodeticCoordinates.AddAltitude(builder, altitude)
    GeodeticCoordinates.AddLatitude(builder, latitude)
    GeodeticCoordinates.AddLongitude(builder, longitude)
    gc = GeodeticCoordinates.End(builder)

    ProjectCreation.Start(builder)
    ProjectCreation.AddMapFrameId(builder, frameIdBuf)
    ProjectCreation.AddName(builder, nameBuf)
    ProjectCreation.AddGeodeticPosition(builder, gc)
    projectCreationMsg = ProjectCreation.End(builder)
    builder.Finish(projectCreationMsg)

    buf = builder.Output()

    responseBuf = stubMeta.CreateProject(bytes(buf))
    return responseBuf


def createProject(
    channel: Channel,
    builder: Builder,
    name: str,
    frameId: str,
    coordSys: str,
    altitude: float,
    latitude: float,
    longitude: float,
) -> str:
    """
    Create a project from the parameters.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        name: The name of the project
        frameId: The coordinate frame of the project
        coordSys: The coordinate system type as a [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        latitude: The latitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        longitude: The longitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)

    Returns:
        The UUID of the created project
    """
    return (
        ProjectInfo.ProjectInfo.GetRootAs(
            createProjectRaw(
                channel,
                builder,
                name,
                frameId,
                coordSys,
                altitude,
                latitude,
                longitude,
            )
        )
        .Uuid()
        .decode("utf-8")
    )


def getOrCreateProject(
    builder: Builder,
    channel: Channel,
    name: str,
    create: bool = True,
    mapFrameId: str = "map",
    coordSys: str = "",
    altitude: float = 0.0,
    latitude: float = 0.0,
    longitude: float = 0.0,
) -> str:
    """
    Get the project, or if not present, create one.

    Args:
        builder: A flatbuffers Builder
        channel: The gRPC channel
        name: The name of the project
        create: Whether to create the project if it does not exist
        mapFrameId: The coordinate frame of the project
        coordSys: The coordinate system type as a [proj ellipsoid](https://proj.org/en/stable/usage/ellipsoids.html#built-in-ellipsoid-definitions)\
            code
        altitude: The altitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        latitude: The latitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)
        longitude: The longitude of the projects coordinate frame origin\
            position on the globe (according to coordSys)

    Returns:
        The UUID of the project
    """
    projectUuid = getProject(builder, channel, name)

    if projectUuid is None:
        if create:
            projectUuid = createProject(
                channel,
                builder,
                name,
                mapFrameId,
                coordSys,
                altitude,
                latitude,
                longitude,
            )
        else:
            sys.exit()

    return projectUuid


def createEmpty(builder: Builder) -> bytes:
    """
    Create an empty flatbuffer

    Args:
        builder: A flatbuffers Builder

    Returns:
        A bytearray representing the empty flatbuffer
    """
    Empty.Start(builder)
    emptyMsg = Empty.End(builder)
    builder.Finish(emptyMsg)
    return bytes(builder.Output())


def deleteProject(
    channel: Channel, builder: Builder, projectName: str, projectUuid: str
):
    """
    Delete a project.

    Args:
        channel: The gRPC channel
        builder: A flatbuffers Builder
        projectUuid: The UUID of the project to delete
    """
    stub = metaOperations.MetaOperationsStub(channel)
    projInfoMsg = createProjectInfo(builder, projectName, projectUuid)
    builder.Finish(projInfoMsg)
    buf = builder.Output()
    stub.DeleteProject(bytes(buf))


def createTimeStamp(
    builder: Builder, seconds: int, nanoseconds: int = 0
) -> int:
    """
    Create a time stamp in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        seconds: The seconds since epoch
        nanoseconds: The nanoseconds since the last second

    Returns:
        A pointer to the constructed timestamp object
    """
    Timestamp.Start(builder)
    Timestamp.AddSeconds(builder, seconds)
    Timestamp.AddNanos(builder, nanoseconds)
    return Timestamp.End(builder)


def createHeader(
    builder: Builder,
    timeStamp: int = None,
    frame: str = None,
    projectUuid: str = None,
    msgUuid: str = None,
) -> int:
    """
    Creates a message header in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        timeStamp: The pointer to a constructed Timestamp object
        frame: The coordinate frame of the message
        projectUuid: The UUID of the project the message belongs to
        msgUuid: The UUID of the message (when not provided it will be set
        by the server)

    Returns:
        A pointer to the constructed header object
    """
    if frame is not None:
        frameStr = builder.CreateString(frame)
    if projectUuid is not None:
        projectUuidStr = builder.CreateString(projectUuid)
    if msgUuid is not None:
        msgUuidStr = builder.CreateString(msgUuid)
    Header.Start(builder)
    if frame is not None:
        Header.AddFrameId(builder, frameStr)
    if timeStamp is not None:
        Header.AddStamp(builder, timeStamp)
    if projectUuid is not None:
        Header.AddUuidProject(builder, projectUuidStr)
    if msgUuid is not None:
        Header.AddUuidMsgs(builder, msgUuidStr)
    return Header.End(builder)


# Point clouds
def createPointField(
    builder: Builder, name: str, offset: int, datatype: int, count: int
) -> int:
    """
    Creates a point field to describe the structure of a point entry in the
    Pointcloud2 message.
    This esssentially mimicks the ROS\
    [Pointfield](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointField.html)\
    message.

    Args:
        builder: A flatbuffers Builder
        name: The name of the point field
        offset: Offset from start of the point struct
        datatype: Datatype used for the point entries\
            (see [fbs definition](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        count: Number of elements in the point field

    Returns:
        A pointer to the constructed point field object
    """
    nameStr = builder.CreateString(name)
    PointField.Start(builder)
    PointField.AddName(builder, nameStr)
    PointField.AddOffset(builder, offset)
    PointField.AddDatatype(builder, datatype)
    PointField.AddCount(builder, count)
    return PointField.End(builder)


def createPointFields(
    builder: Builder,
    channels: List[str],
    datatype: int,
    dataTypeOffset: int,
    count: int,
) -> List[int]:
    """
    Creates point fields for all specified channels.

    Args:
        builder: A flatbuffers Builder
        channels: List of channel names
        datatype: Datatype used for the point entries\
            (see [fbs definition](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/point_field.fbs))
        dataTypeOffset: Offset from start of the point struct, in this case the
        offset is used for every channel
        count: Number of elements in each point field

    Returns:
        A list of pointers to the constructed point field objects
    """
    pointFieldsList = []
    offset = 0
    for channel in channels:
        pointFieldsList.append(
            createPointField(builder, channel, offset, datatype, count)
        )
        offset += dataTypeOffset
    return pointFieldsList


def createPoint2d(builder: Builder, x: float, y: float) -> int:
    """
    Creates a 2D point in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        x: The x-coordinate of the 2D point
        y: The y-coordinate of the 2D point

    Returns:
        A pointer to the constructed point object
    """
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    return Point.End(builder)


def createPoint(builder: Builder, x: float, y: float, z: float) -> int:
    """
    Creates a 3D point in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        x: The x-coordinate of the 3D point
        y: The y-coordinate of the 3D point
        z: The z-coordinate of the 3D point

    Returns:
        A pointer to the constructed point object
    """
    Point.Start(builder)
    Point.AddX(builder, x)
    Point.AddY(builder, y)
    Point.AddZ(builder, z)
    return Point.End(builder)


def createPointStamped(
    builder: Builder, point: int, header: int, labelGeneralCategoryVector: int
) -> int:
    """
    Creates a 3D point stamped in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        point: The pointer to a 3D point object
        header: The pointer to the header object of the point
        labelGeneralCategoryVector: The pointer to a vector of\
            [LabelsWithInstanceWithCategory](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/labels_with_category.fbs)\
            objects of the point

    Returns:
        A pointer to the constructed point object
    """

    PointStamped.Start(builder)
    PointStamped.AddPoint(builder, point)
    PointStamped.AddHeader(builder, header)
    PointStamped.AddLabelsGeneral(builder, labelGeneralCategoryVector)
    return PointStamped.End(builder)


def createBoundingBox(
    builder: Builder,
    centerPoint: int,
    spatialExtent: int,
    rotation: Union[int, None] = None,
) -> int:
    """
    Creates a 3D bounding box in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        centerPoint: The pointer to the 3D Point object of the center point of
        the bounding box spatialExtent: The pointer to the 3D Point object
        representing the spatial extent in x, y and z direction rotation:
        The rotation of the bounding box as a flatbuffers Quaternion

    Returns:
        A pointer to the constructed\
        [BoundingBox](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/boundingbox.fbs)
        object
    """
    Boundingbox.Start(builder)
    Boundingbox.AddCenterPoint(builder, centerPoint)
    Boundingbox.AddSpatialExtent(builder, spatialExtent)
    if rotation is not None:
        Boundingbox.AddRotation(builder, rotation)
    return Boundingbox.End(builder)


def createPolygon2D(
    builder: Builder, height: float, z: float, vertices: List[int]
) -> int:
    """
    Create a 2D Polygon in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        height: The height of the polygon in meter.
        z: The z-coordinate of the polygon from which the height is measured
        vertices: A list of pointers to\
            [Point2D](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/point2d.fbs)\
            objects as the vertices of the polygon, where the order of the\
            points in the list is such that the points are arranged in a circle.

    Returns:
        A pointer to the constructed\
        [Polygon2D](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/polygon2d.fbs)
    """

    Polygon2D.StartVerticesVector(builder, len(vertices))
    for v in vertices:
        builder.PrependUOffsetTRelative(v)
    vertices_fb = builder.EndVector()

    Polygon2D.Start(builder)
    Polygon2D.AddHeight(builder, height)
    Polygon2D.AddZ(builder, z)
    Polygon2D.AddVertices(builder, vertices_fb)

    return Polygon2D.End(builder)


def addToBoundingBoxLabeledVector(
    builder: Builder, boundingBoxLabeledList: List[int]
) -> int:
    """
    Adds list of boudingBoxLabeled into the labelsBbVector
    of a flatbuffers pointcloud2.

    Args:
        builder: A flatbuffers Builder
        boundingBoxLabeledList: A list of pointers to\
            [BoundingBoxLabeledWithCategory](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/boundingbox_labeled_with_category.fbs)

    Returns:
        A pointer to the constructed vector of bounding box labeled
        with category objects
    """
    PointCloud2.StartLabelsBbVector(builder, len(boundingBoxLabeledList))
    for bb in reversed(boundingBoxLabeledList):
        builder.PrependUOffsetTRelative(bb)
    return builder.EndVector()


def addToPointFieldVector(builder: Builder, pointFieldList: List[int]) -> int:
    """
    Adds a list of pointFields into the fieldsVector of a flatbuffers
    pointcloud2.

    Args:
        builder: A flatbuffers Builder
        pointFieldList: A list of pointers to\
            [PointField](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/point_field.fbs)
            objects

    Returns:
        A pointer to the constructed vector of point fields objects
    """
    PointCloud2.StartFieldsVector(builder, len(pointFieldList))
    for pointField in reversed(pointFieldList):
        builder.PrependUOffsetTRelative(pointField)
    return builder.EndVector()


def createQuery(
    builder: Builder,
    timeIntervalVector: Union[int, None] = None,
    labels: Union[List[int], None] = None,
    mustHaveAllLabels: bool = False,
    projectUuids: List[str] = None,
    instanceUuids: List[str] = None,
    dataUuids: List[str] = None,
    withoutData: bool = False,
    polygon2d: Union[int, None] = None,
    polygon2dSensorPos: Union[int, None] = None,
    fullyEncapsulated: bool = False,
    inMapFrame: bool = True,
    sortByTime: bool = False,
) -> int:
    """
    Create a query, all parameters are optional.

    Args:
        builder: A flatbuffers Builder
        timeIntervalVector: The pointer to vector of TimeInterval objects
        representing the time frames
        labels: A list of pointers to\
            [LabelsWithInstanceWithCategory](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/labels_with_instance_with_category.fbs)\
            flatbuffers objects, which the instances should at least have one of
        mustHaveAllLabels: A boolean indicating if the returned instances should
        have all the labels each
        projectUuids: A list of project UUIDs to execute the query on
        instanceUuids: A list of specific instance UUIDs to query
        dataUuids: A list of specific data UUIDs to query
        withoutData: A boolean indicating if the query should return instances
        without their attached data
        polygon2d: A pointer to a Polygon2D object to retrieve only instances
        within the polygon
        polygon2dSensorPos: A pointer to a Polygon2D object to retrieve only
        instances captured by sensors within the polygon
        fullyEncapsulated: A boolean indicating if the returned instances should
        be fully encapsulated by the polygon
        inMapFrame: A boolean indicating if the polygon coordinates are in the
        map frame or in EPSG world coordinates
        sortByTime: A boolean indicating if the returned instances should be
        sorted by time

    Returns:
        A pointer to the constructed query object
    """

    if projectUuids is not None:
        if isinstance(projectUuids, str):
            projectUuids = [projectUuids]
        # serialize strings
        projectUuidsSerialized = [
            builder.CreateString(projectUuid) for projectUuid in projectUuids
        ]
        Query.StartProjectuuidVector(builder, len(projectUuidsSerialized))
        for projectUuid in reversed(projectUuidsSerialized):
            builder.PrependUOffsetTRelative(projectUuid)
        projectUuidsOffset = builder.EndVector()

    if instanceUuids is not None:
        # serialize strings
        instanceUuidsSerialized = [
            builder.CreateString(instanceUuid) for instanceUuid in instanceUuids
        ]
        Query.StartInstanceuuidVector(builder, len(instanceUuidsSerialized))
        for instance in reversed(instanceUuidsSerialized):
            builder.PrependUOffsetTRelative(instance)
        instanceOffset = builder.EndVector()

    if dataUuids is not None:
        # serialize strings
        dataUuidsSerialized = [
            builder.CreateString(dataUuid) for dataUuid in dataUuids
        ]
        Query.StartDatauuidVector(builder, len(dataUuidsSerialized))
        for dataUuid in reversed(dataUuidsSerialized):
            builder.PrependUOffsetTRelative(dataUuid)
        dataUuidOffset = builder.EndVector()

    if labels is not None:
        Query.QueryStartLabelVector(builder, len(labels))
        for label in reversed(labels):
            builder.PrependUOffsetTRelative(label)
        labelOffset = builder.EndVector()

    Query.Start(builder)
    if polygon2d is not None:
        Query.AddPolygon(builder, polygon2d)
    if polygon2dSensorPos is not None:
        Query.AddPolygonSensorPosition(builder, polygon2dSensorPos)
    if timeIntervalVector is not None:
        Query.AddTimeintervals(builder, timeIntervalVector)
    if labels is not None:
        Query.AddLabel(builder, labelOffset)
    # no if; has default value
    Query.AddMustHaveAllLabels(builder, mustHaveAllLabels)
    if projectUuids is not None:
        Query.AddProjectuuid(builder, projectUuidsOffset)
    if instanceUuids is not None:
        Query.AddInstanceuuid(builder, instanceOffset)
    if dataUuids is not None:
        Query.QueryAddDatauuid(builder, dataUuidOffset)
    # no if; has default value
    Query.AddWithoutdata(builder, withoutData)
    Query.AddFullyEncapsulated(builder, fullyEncapsulated)
    Query.AddInMapFrame(builder, inMapFrame)
    Query.AddSortByTime(builder, sortByTime)

    return Query.End(builder)


def createQueryInstance(builder: Builder, query: int, datatype: int) -> int:
    """
    Create a query for instances.

    Args:
        builder: A flatbuffers Builder
        query: The pointer to the query object
        datatype: The [Datatype](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instances to query

    Returns:
        A pointer to the constructed query instance object
    """
    QueryInstance.Start(builder)
    QueryInstance.AddDatatype(builder, datatype)
    QueryInstance.AddQuery(builder, query)
    return QueryInstance.End(builder)


def createTimeIntervalVector(
    builder: Builder, timeMinList: List[int], timeMaxList: List[int]
) -> int:
    """
    Create a vector of closed time intervals in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        timeMinList: The list of pointers to the Timestamp objects representing
            the lower bound of the time of the intervals
        timeMaxList: The list of pointers to the Timestamp objects representing
            the upper bound of the time of the intervals

    Returns:
        A pointer to the constructed time interval vector object
    """
    intervals = []
    for timeMin, timeMax in zip(timeMinList, timeMaxList):
        intervals.append(createTimeInterval(builder, timeMin, timeMax))

    Query.StartTimeintervalsVector(builder, len(intervals))
    for interval in intervals:
        builder.PrependUOffsetTRelative(interval)

    return builder.EndVector()


def createTimeInterval(builder: Builder, timeMin: int, timeMax: int) -> int:
    """
    Create a closed time interval in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        timeMin: The pointer to a Timestamp object representing
            the lower bound of the time of the interval
        timeMax: The pointer to a Timestamp object representing
            the upper bound of the time of the interval

    Returns:
        A pointer to the constructed time interval object
    """
    TimeInterval.Start(builder)
    TimeInterval.AddTimeMin(builder, timeMin)
    TimeInterval.AddTimeMax(builder, timeMax)
    return TimeInterval.End(builder)


def createTransformStampedQuery(
    builder: Builder, header: int, childFrameId: str
) -> int:
    """
    Create a transform stamped query in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        header: The pointer to a header object including the parent frame id
        childFrameId: The child frame id

    Returns:
        A pointer to the constructed transform stamped query object
    """
    child_frame_obj = builder.CreateString(childFrameId)
    TransformStampedQuery.Start(builder)
    TransformStampedQuery.AddHeader(builder, header)
    TransformStampedQuery.AddChildFrameId(builder, child_frame_obj)
    return TransformStampedQuery.End(builder)


def createTransformStampedQueryInterval(
    builder: Builder,
    header: int,
    childFrameId: str,
    time_min: int,
    time_max: int,
) -> int:
    """
    Create a transform stamped query interval object in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        header: The pointer to a header object including the parent frame id
        childFrameId: The child frame id
        time_min: the pointer to the earliest inclusive time stamp
            flatbuffers object to span the time_interval
        time_max: the pointer to the latest exclusive time stamp
            flatbuffers object to span the time_interval

    Returns:
        A pointer to the constructed transform stamped query interval object
    """
    tf_stamped_query: int = createTransformStampedQuery(
        builder, header, childFrameId
    )
    time_interval: int = createTimeInterval(builder, time_min, time_max)

    TransformStampedIntervalQuery.Start(builder)
    TransformStampedIntervalQuery.AddTransformStampedQuery(
        builder, tf_stamped_query
    )
    TransformStampedIntervalQuery.AddTimeInterval(builder, time_interval)

    return TransformStampedIntervalQuery.End(builder)


def createRegionOfInterest(
    builder: Builder,
    x_offset: int,
    y_offset: int,
    height: int,
    width: int,
    do_rectify: bool,
) -> int:
    """
    Create a region of interest in flatbuffers. This has the same structure as
    the ROS message\
    [RegionOfInterest](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RegionOfInterest.html)
    type.

    Args:
        builder: A flatbuffers Builder
        x_offset: Leftmost pixel of the region
        y_offset: Topmost pixel of the region
        height: The height of the region
        width: The width of the region
        do_rectify: A boolean indicating if a distinct ROI should be calculated

    Returns:
        A pointer to the constructed region of interest object
    """
    RegionOfInterest.Start(builder)
    RegionOfInterest.AddXOffset(builder, x_offset)
    RegionOfInterest.AddYOffset(builder, y_offset)
    RegionOfInterest.AddHeight(builder, height)
    RegionOfInterest.AddWidth(builder, width)
    RegionOfInterest.AddDoRectify(builder, do_rectify)
    return RegionOfInterest.End(builder)


def createCameraIntrinsics(
    builder: Builder,
    header: int,
    height: int,
    width: int,
    distortion_model: str,
    distortion: List[float],
    intrinsics_matrix: List[float],
    rectification_matrix: List[float],
    projection_matrix: List[float],
    binning_x: int,
    binning_y: int,
    region_of_interest: int,
    max_viewing_dist: float,
) -> int:
    """
    Create a camera intrinsics object in flatbuffers.
    This is structured in the same way as the ROS message\
    [CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html).

    Args:
        builder: A flatbuffers Builder
        header: The pointer to the header object of the camera intrinsics
        height: The height of the camera image
        width: The width of the camera image
        distortion_model: The distortion model of the camera
        distortion: The distortion coefficients of the camera
        intrinsics_matrix: The intrinsics matrix of the camera
        rectification_matrix: The rectification matrix of the camera
        projection_matrix: The projection matrix of the camera
        binning_x: The binning in horizontal direction
        binning_y: The binning in vertical direction
        region_of_interest: The region of interest of the camera
        max_viewing_dist: The maximum viewing distance of the camera

    Returns:
        A pointer to the constructed camera intrinsics object.
    """
    dm_buf = builder.CreateString(distortion_model)

    CameraIntrinsics.StartDistortionVector(builder, len(distortion))
    for d in reversed(distortion):
        builder.PrependFloat64(d)
    distortionOffset = builder.EndVector()

    CameraIntrinsics.StartIntrinsicMatrixVector(builder, len(intrinsics_matrix))
    for im in reversed(intrinsics_matrix):
        builder.PrependFloat64(im)
    IMOffset = builder.EndVector()

    CameraIntrinsics.StartRectificationMatrixVector(
        builder, len(rectification_matrix)
    )
    for rm in reversed(rectification_matrix):
        builder.PrependFloat64(rm)
    RMOffset = builder.EndVector()

    CameraIntrinsics.StartProjectionMatrixVector(
        builder, len(projection_matrix)
    )
    for pm in reversed(projection_matrix):
        builder.PrependFloat64(pm)
    PMOffset = builder.EndVector()

    CameraIntrinsics.Start(builder)
    CameraIntrinsics.AddHeader(builder, header)
    CameraIntrinsics.AddHeight(builder, height)
    CameraIntrinsics.AddWidth(builder, width)
    CameraIntrinsics.AddDistortionModel(builder, dm_buf)
    CameraIntrinsics.AddDistortion(builder, distortionOffset)
    CameraIntrinsics.AddIntrinsicMatrix(builder, IMOffset)
    CameraIntrinsics.AddRectificationMatrix(builder, RMOffset)
    CameraIntrinsics.AddProjectionMatrix(builder, PMOffset)
    CameraIntrinsics.AddBinningX(builder, binning_x)
    CameraIntrinsics.AddBinningY(builder, binning_y)
    CameraIntrinsics.AddRegionOfInterest(builder, region_of_interest)
    CameraIntrinsics.AddMaximumViewingDistance(builder, max_viewing_dist)

    return CameraIntrinsics.End(builder)


def createCameraIntrinsicsQuery(
    builder: Builder, ci_uuid: str, project_uuid: str
) -> str:
    """
    Create a query for camera intrinsics objects in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        ci_uuid: The UUID of the camera intrinsics
        project_uuid: The UUID of the project to query the camera intrinsics
        from

    Returns:
        A pointer to the constructed camera intrinsics query object
    """
    ci_uuid_str = builder.CreateString(ci_uuid)
    project_uuid_str = builder.CreateString(project_uuid)
    CameraIntrinsicsQuery.Start(builder)
    CameraIntrinsicsQuery.AddUuidCameraIntrinsics(builder, ci_uuid_str)
    CameraIntrinsicsQuery.AddUuidProject(builder, project_uuid_str)

    return CameraIntrinsicsQuery.End(builder)


def createUuidDatatypePair(builder: Builder, uuid: str, datatype: int) -> int:
    """
    Create a UUID datatype pair in flatbuffers for meta type services.

    Args:
        builder: A flatbuffers Builder
        uuid: The UUID of a project to retrieve the information from
        datatype: The [Datatype](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instance to retrieve the information from

    Returns:
        A pointer to the constructed UUID datatype pair object
    """
    uuidStr = builder.CreateString(uuid)

    UuidDatatypePair.Start(builder)
    UuidDatatypePair.AddProjectuuid(builder, uuidStr)
    UuidDatatypePair.AddDatatype(builder, datatype)
    return UuidDatatypePair.End(builder)


def createUuidDatatypeWithCategory(
    builder: Builder, uuid: str, datatype: int, category: str
) -> int:
    """
    Create a UUID datatype pair with a category in flatbuffers for meta type
    services.

    Args:
        builder: A flatbuffers Builder
        uuid: The UUID of a project to retrieve the information from
        datatype: The [Datatype](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/datatype.fbs)\
            of the instance to retrieve the information from
        category: The category to which the instance belongs

    Returns:
        A pointer to the constructed UUID datatype with category object
    """
    categoryStr = builder.CreateString(category)

    UuidDatatypePair = createUuidDatatypePair(builder, uuid, datatype)

    UuidDatatypeWithCategory.Start(builder)
    UuidDatatypeWithCategory.AddCategory(builder, categoryStr)
    UuidDatatypeWithCategory.AddUuidAndDatatype(builder, UuidDatatypePair)
    return UuidDatatypeWithCategory.End(builder)


def createProjectInfo(builder: Builder, name: str, uuid: str) -> int:
    """
    Create a project info object in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        name: The name of the project
        uuid: The UUID of the project

    Returns:
        A pointer to the constructed project info object
    """
    nameStr = builder.CreateString(name)
    uuidStr = builder.CreateString(uuid)

    ProjectInfo.Start(builder)
    ProjectInfo.AddName(builder, nameStr)
    ProjectInfo.AddUuid(builder, uuidStr)
    return ProjectInfo.End(builder)


def createVector3(builder: Builder, t: Tuple[float, float, float]) -> int:
    """
    Create a 3D vector in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        t: The 3D vector to be created

    Returns:
        A pointer to the constructed 3D vector object
    """
    Vector3.Start(builder)
    Vector3.AddX(builder, t[0])
    Vector3.AddY(builder, t[1])
    Vector3.AddZ(builder, t[2])
    return Vector3.End(builder)


def createQuaternion(builder: Builder, quat: Quaternion.Quaternion) -> int:
    """
    Create a quaternion in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        quat: The quaternion to be created

    Returns:
        A pointer to the constructed quaternion object
    """
    Quaternion.Start(builder)
    Quaternion.AddX(builder, quat.x)
    Quaternion.AddY(builder, quat.y)
    Quaternion.AddZ(builder, quat.z)
    Quaternion.AddW(builder, quat.w)
    return Quaternion.End(builder)


def createTransform(builder: Builder, t: int, quat: int) -> int:
    """
    Create a transform in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        t: A pointer to a Vector3 representing the translation of the transform
        quat: The pointer to the Quaternion flatbuffers object of the transform

    Returns:
        A pointer to the constructed transform object
    """
    Transform.Start(builder)
    Transform.AddTranslation(builder, t)
    Transform.AddRotation(builder, quat)
    return Transform.End(builder)


def createTransformStamped(
    builder: Builder,
    childFrame: str,
    headerTf: int,
    transform: int,
    isStatic=False,
) -> int:
    """
    Create a stamped transform in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        childFrame: The child frame of the transform
        headerTf: The pointer to the flatbuffers header of the transform
        transform: The pointer to the flatbuffers transform
        isStatic: Flag if it is a static tf

    Returns:
        A pointer to the constructed stamped transform object
    """
    childFrame = builder.CreateString(childFrame)
    TransformStamped.Start(builder)
    TransformStamped.AddChildFrameId(builder, childFrame)
    TransformStamped.AddHeader(builder, headerTf)
    TransformStamped.AddTransform(builder, transform)
    TransformStamped.AddIsStatic(builder, isStatic)
    return TransformStamped.End(builder)


def createImage(
    builder: Builder,
    header: int,
    encoding: str,
    is_bigendian: bool,
    camera_intrinsics_uuid: str,
    image: np.ndarray = None,
    height: int = None,
    width: int = None,
    uri: str = None,
    labels: Union[List[int], None] = None,
) -> int:
    """
    Create an image in flatbuffers.

    Args:
        builder: A flatbuffers Builder
        image: The image to be created (can be read from a file using
        imageio.imread())
        header: The pointer to the header object of the image
        encoding: The encoding of the image, e.g. rgb8
        is_bigendian: Bool if the data is big endian
        labelCategory: A list of pointers to\
            [LabelCategory](https://github.com/DFKI-NI/seerep/blob/main/seerep_msgs/fbs/label_category.fbs)\
            objects to attach to the image
        camera_intrinsics_uuid: The uuid of the corresponding camera intrinsic

    Returns:
        A pointer to the constructed image object
    """
    encoding = builder.CreateString(encoding)

    if labels:
        Image.ImageStartLabelsVector(builder, len(labels))
        for label in reversed(labels):
            builder.PrependUOffsetTRelative(label)
        label_offset = builder.EndVector()

    camera_intrinsics_uuid_offset = builder.CreateString(camera_intrinsics_uuid)
    if image is not None:
        data_offset = builder.CreateByteVector(image.tobytes())
    if uri is not None:
        uri_offset = builder.CreateString(uri)

    Image.Start(builder)
    Image.AddHeader(builder, header)
    Image.AddHeight(builder, height)
    Image.AddWidth(builder, width)
    Image.AddEncoding(builder, encoding)
    Image.AddIsBigendian(builder, is_bigendian)
    if image is not None:
        Image.AddStep(builder, image.nbytes // image.shape[0])
        Image.AddData(builder, data_offset)
    if uri is not None:
        Image.AddUri(builder, uri_offset)
    if labels is not None:
        Image.AddLabels(builder, label_offset)
    Image.AddUuidCameraintrinsics(builder, camera_intrinsics_uuid_offset)
    return Image.End(builder)


# TODO: find a way to not use to ifs for the optional parameters
def create_label(
    builder,
    label: str,
    label_id: int,
    instance_uuid: str = None,
    instance_id: int = None,
):
    label_offset = builder.CreateString(label)
    if instance_uuid is not None:
        instance_uuid_offset = builder.CreateString(instance_uuid)
    Label.Start(builder)
    Label.AddLabel(builder, label_offset)
    Label.LabelAddLabelIdDatumaro(builder, label_id)
    if instance_uuid is not None:
        Label.AddInstanceUuid(builder, instance_uuid_offset)
    if instance_id is not None:
        Label.AddInstanceIdDatumaro(builder, instance_id)
    return Label.End(builder)


def create_label_category(
    builder, labels: List[Label.Label], datumaro_json: str, category: str = None
):
    if category is not None:
        category_offset = builder.CreateString(category)
    datumaro_json_offset = builder.CreateString(datumaro_json)

    LabelCategory.StartLabelsVector(builder, len(labels))
    for label in labels:
        builder.PrependUOffsetTRelative(label)
    vec_offset = builder.EndVector()

    LabelCategory.Start(builder)
    if category is not None:
        LabelCategory.AddCategory(builder, category_offset)
    LabelCategory.AddLabels(builder, vec_offset)
    LabelCategory.AddDatumaroJson(builder, datumaro_json_offset)
    return LabelCategory.End(builder)


def create_dataset_uuid_label(
    builder,
    projectUuid: str,
    datasetUuid: str,
    labels: List[LabelCategory.LabelCategory],
):
    DatasetUuidLabel.StartLabelsVector(builder, len(labels))
    for label in labels:
        builder.PrependUOffsetTRelative(label)
    vec_offset = builder.EndVector()

    projectUuidOffset = builder.CreateString(projectUuid)
    datasetUuidOffset = builder.CreateString(datasetUuid)

    DatasetUuidLabel.Start(builder)
    DatasetUuidLabel.AddProjectUuid(builder, projectUuidOffset)
    DatasetUuidLabel.AddDatasetUuid(builder, datasetUuidOffset)
    DatasetUuidLabel.AddLabels(builder, vec_offset)
    return DatasetUuidLabel.End(builder)
