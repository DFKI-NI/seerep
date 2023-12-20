from enum import auto
from typing import Dict, List

from flatbuffers import Builder
from grpc import Channel
from seerep.fb import (
    Datatype,
    LabelsWithCategory,
    Polygon2D,
    Query,
    QueryInstance,
    TimeInterval,
)
from seerep.util import fb_helper as fbh
from seerep.util.datastructures import FrozenEnum
from seerep.util.helper.msgs_base import MsgsFb, MsgsFunctions, expect_component
from seerep.util.helper.service_manager import ServiceManager

C_MAX_INTEGER: int = 2147483647


class EnumFbQuery(FrozenEnum):
    POLYGON = auto()  # def: None
    FULLY_ENCAPSULATED = auto()  # def: False
    IN_MAP_FRAME = auto()  # def: True
    TIMEINTERVAL = auto()  # def: None
    LABEL = auto()  # def: None
    SPARQL_QUERY = auto()  # def: None
    ONTOLOGY_URI = auto()  # def: None
    MUST_HAVE_ALL_LABELS = auto()  # def: False
    PROJECTUUID = auto()  # def: None
    INSTANCEUUID = auto()  # def: None
    DATAUUID = auto()  # def: None
    WITHOUTDATA = auto()  # def: False
    MAX_NUM_DATA = auto()  # def: None


class EnumFbQueryInstance(FrozenEnum):
    DATATYPE = auto()  # def: None
    QUERY = auto()  # def: None


class FbQuery(MsgsFb[Query.Query]):
    def _set_enum_func_mapping(self) -> Dict[EnumFbQuery, MsgsFunctions]:
        return {
            EnumFbQuery.POLYGON: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.polygon2D(self.builder)
            ),
            EnumFbQuery.FULLY_ENCAPSULATED: MsgsFunctions(lambda: False, lambda: True),
            EnumFbQuery.IN_MAP_FRAME: MsgsFunctions(lambda: True, lambda: False),
            EnumFbQuery.TIMEINTERVAL: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.time_interval(self.builder)
            ),
            EnumFbQuery.LABEL: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.label_with_category(self.builder)
            ),
            EnumFbQuery.SPARQL_QUERY: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.sparql_query(self.builder)
            ),
            EnumFbQuery.ONTOLOGY_URI: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.ontology_uri(self.builder)
            ),
            EnumFbQuery.MUST_HAVE_ALL_LABELS: MsgsFunctions(
                lambda: False, lambda: True
            ),
            EnumFbQuery.PROJECTUUID: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.projectuuid(self.builder, self.channel)
            ),
            EnumFbQuery.INSTANCEUUID: MsgsFunctions(
                lambda: None, lambda: self.instanceuuid()
            ),
            EnumFbQuery.DATAUUID: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.datauuid(self.builder, self.channel)
            ),
            EnumFbQuery.WITHOUTDATA: MsgsFunctions(lambda: False, lambda: True),
            EnumFbQuery.MAX_NUM_DATA: MsgsFunctions(
                lambda: None, lambda: Dtypes.Fb.max_num_data()
            ),
        }

    @expect_component(EnumFbQuery.PROJECTUUID)
    def instanceuuid(self) -> List[int]:
        return Dtypes.Fb.intanceuuid(
            self.builder, self.channel, self.get_component(EnumFbQuery.PROJECTUUID)
        )

    @expect_component(EnumFbQuery.PROJECTUUID)
    def datauuid(self) -> List[int]:
        return Dtypes.Fb.datauuid(
            self.builder, self.channel, self.get_component(EnumFbQuery.DATAUUID)
        )

    def _assemble_datatype_instance(self):
        polygon = self.get_component(EnumFbQuery.POLYGON)
        fully_encapsulated = self.get_component(EnumFbQuery.FULLY_ENCAPSULATED)
        in_map_frame = self.get_component(EnumFbQuery.IN_MAP_FRAME)
        timeinterval = self.get_component(EnumFbQuery.TIMEINTERVAL)
        label = self.get_component(EnumFbQuery.LABEL)
        sparql_query = self.get_component(EnumFbQuery.SPARQL_QUERY)
        ontology_uri = self.get_component(EnumFbQuery.ONTOLOGY_URI)
        must_have_all_labels = self.get_component(EnumFbQuery.MUST_HAVE_ALL_LABELS)
        projectuuid = self.get_component(EnumFbQuery.PROJECTUUID)
        instanceuuid = self.get_component(EnumFbQuery.INSTANCEUUID)
        datauuid = self.get_component(EnumFbQuery.DATAUUID)
        withoutdata = self.get_component(EnumFbQuery.WITHOUTDATA)
        max_num_data = self.get_component(EnumFbQuery.MAX_NUM_DATA)

        return fbh.createQuery(
            self.builder,
            timeinterval,
            label,
            must_have_all_labels,
            projectuuid,
            instanceuuid,
            datauuid,
            withoutdata,
            polygon,
            fully_encapsulated,
            in_map_frame,
        )


class FbQueryInstance(MsgsFb[QueryInstance.QueryInstance]):
    def _set_enum_func_mapping(self) -> Dict[FrozenEnum, MsgsFunctions]:
        return {
            EnumFbQueryInstance.DATATYPE: MsgsFunctions(
                lambda: Datatype.Datatype().All, lambda: Datatype.Datatype().PointCloud
            ),
            EnumFbQueryInstance.QUERY: MsgsFunctions(
                lambda: Dtypes.FbDefaults.query(self.channel), lambda: self.query()
            ),
        }

    def query(self):
        features = set(
            [EnumFbQuery.WITHOUTDATA, EnumFbQuery.IN_MAP_FRAME, EnumFbQuery.PROJECTUUID]
        )
        return FbQuery(self.channel, self.builder, features).datatype_instance

    def _assemble_datatype_instance(self):
        datatype = self.get_component(EnumFbQueryInstance.DATATYPE)
        query = self.get_component(EnumFbQueryInstance.QUERY)

        QueryInstance.Start(self.builder)
        QueryInstance.AddDatatype(self.builder, datatype)
        QueryInstance.AddQuery(self.builder, query)
        return QueryInstance.End(self.builder)


class DatatypeImplementations:
    class Fb:
        @classmethod
        def polygon2D(cls, builder: Builder) -> Polygon2D.Polygon2D:
            polygon_vertices = []
            polygon_vertices.append(fbh.createPoint2d(builder, 0, 0))
            polygon_vertices.append(fbh.createPoint2d(builder, 0, 100))
            polygon_vertices.append(fbh.createPoint2d(builder, 100, 100))
            polygon_vertices.append(fbh.createPoint2d(builder, 100, 0))
            return fbh.createPolygon2D(builder, 100, 0, polygon_vertices)

        @classmethod
        def time_interval(cls, builder: Builder) -> TimeInterval.TimeInterval:
            timeMin = fbh.createTimeStamp(builder, 1610549273, 0)
            timeMax = fbh.createTimeStamp(builder, 1938549273, 0)
            return fbh.createTimeInterval(builder, timeMin, timeMax)

        @classmethod
        def label_with_category(
            cls, builder: Builder
        ) -> LabelsWithCategory.LabelsWithCategory:
            category = ["0"]
            labels = {
                "0": [
                    (builder.CreateString("testlabel0"), 0.3),
                    (builder.CreateString("testlabelgeneral0"), 0.7),
                ]
            }
            return fbh.createLabelsWithCategories(builder, category, labels)

        @classmethod
        def sparql_query(cls, builder: Builder):
            return None

        @classmethod
        def ontology_uri(cls, builder: Builder):
            return None

        @classmethod
        def projectuuid(cls, builder: Builder, channel: Channel) -> List[int]:
            return [
                builder.CreateString(fbh.getProject(builder, channel, "testproject"))
            ]

        @classmethod
        # returns list of flatbuffered string, those are registered as ints
        def intanceuuid(
            cls, builder: Builder, channel: Channel, proj_uuid: str
        ) -> List[int]:
            query_instance = FbQueryInstance(channel).datatype_instance
            serv_man = ServiceManager(channel)
            uuids_pp = serv_man.call_get_instances_fb(builder, query_instance)

            uuids_by_proj = [
                uuids_pp.UuidsPerProject(i)
                for i in range(uuids_pp.UuidsPerProjectLength())
                if uuids_pp.UuidsPerProject(i).ProjectUuid == proj_uuid
            ]

            # return every second uuid
            uuids = [
                builder.CreateString(uuids_by_proj[0].Uuids(i).decode())
                for i in range(0, uuids_by_proj[0].UuidsLength(), 2)
            ]

            return uuids

        @classmethod
        def datauuid(cls, builder: Builder, channel: Channel) -> List[int]:
            query_fb = FbQuery(channel).datatype_instance
            serv_man = ServiceManager(channel)
            images = serv_man.call_get_images_fb(builder, query_fb)
            points = serv_man.call_get_points_fb(builder, query_fb)
            pcl2s = serv_man.call_get_pointcloud2_fb(builder, query_fb)

            image_uuids = [image.Header().UuidMsgs().decode() for image in images]
            point_uuids = [point.Header().UuidMsgs().decode() for point in points]
            pcl2_uuids = [pcl.Header().UuidMsgs().decode() for pcl in pcl2s]

            # debugging
            print("images: " + str(image_uuids))
            print("points: " + str(point_uuids))
            print("pcls: " + str(pcl2_uuids))

            # fill up return list with every second uuid
            ret_lst = [
                builder.CreateString(image_uuids[i])
                for i in range(0, len(image_uuids), 2)
            ]
            ret_lst += [
                builder.CreateString(point_uuids[i])
                for i in range(0, len(point_uuids), 2)
            ]
            ret_lst += [
                builder.CreateString(pcl2_uuids[i])
                for i in range(0, len(pcl2_uuids), 2)
            ]

            return ret_lst

        @classmethod
        def max_num_data(cls, builder: Builder):
            return 2

    class FbDefaults:
        @classmethod
        def query(cls, channel: Channel) -> Query:
            return FbQuery(channel).datatype_instance


# shorten datatype definition
Dtypes = DatatypeImplementations
