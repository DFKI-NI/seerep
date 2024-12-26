#ifndef SEEREP_CORE_CORE_DATASET_H_
#define SEEREP_CORE_CORE_DATASET_H_

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>

#include <optional>
#include <unordered_set>

// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/datatype.h>
#include <seerep_msgs/query.h>

// seerep_hdf5_core
#include <seerep_hdf5_core/hdf5_core_datatype_interface.h>

// seerep_core
#include "core_instances.h"
#include "core_tf.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

typedef CGAL::Exact_predicates_exact_constructions_kernel ExactKernel;
typedef CGAL::Point_3<ExactKernel> CGPoint_3;
typedef CGAL::Surface_mesh<CGPoint_3> CGSurfaceMesh;
typedef CGAL::Polygon_2<ExactKernel> CGPolygon_2;
typedef CGAL::Point_2<ExactKernel> CGPoint_2;
typedef CGSurfaceMesh::Face_index CGFaceIdx;

namespace seerep_core
{
/**
 * @brief This is the class handling the spatio-temporal-semantic indexing of a
 * dataset
 *
 * a dataset can be added to the indices and spatio-temporal-semantic queries
 * can be executed to get the UUIDs of the datasets matching the query. On
 * startup all datasets are loaded from the HDF5 file Datasets added during
 * runtime must be added to the indices via the corresponding public method
 */
class CoreDataset
{
private:
  /** @brief a struct for the variables that are needed for each datatype */
  struct DatatypeSpecifics
  {
    /** @brief shared pointer to the object handling the HDF5 io for the datatype */
    std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5io;
    /** @brief vector of the datasets which couldn't be added to the spatial
     * index yet due to missing tf*/
    std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>
        dataWithMissingTF =
            std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>();
    /** @brief the spatial r-tree of the spatial extent of the sensor data for
     * the spatial index*/
    seerep_core_msgs::rtree rt = seerep_core_msgs::rtree();
    /** @brief the spatial r-tree of the sensor position for the spatial index*/
    seerep_core_msgs::rtree rtSensorPos = seerep_core_msgs::rtree();
    /** @brief the temporal r-tree for the temporal index*/
    seerep_core_msgs::timetree timetree = seerep_core_msgs::timetree();
    /** @brief map from the category of labels to the map from label to the
     * UUIDs of the datasets annotated with this label*/
    std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>
        categoryLabelDatasetsMap = std::unordered_map<
            std::string,
            std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>();
    /** @brief map from the UUID of the dataset a vector of UUID of instances
     * the dataset is showing */
    std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                       boost::hash<boost::uuids::uuid>>
        datasetInstancesMap =
            std::unordered_map<boost::uuids::uuid,
                               std::vector<boost::uuids::uuid>,
                               boost::hash<boost::uuids::uuid>>();
  };

public:
  /**
   * @brief Constructs the object handling the indexing of datasets
   * @param tfOverview pointer to the object handling the transformations
   * @param coreInstances pointer to the object handling the instances
   * @param frameId the common frame id of the project used for the spatial index
   */
  CoreDataset(std::shared_ptr<seerep_core::CoreTf> tfOverview,
              std::shared_ptr<seerep_core::CoreInstances> coreInstances,
              const std::string& frameId);
  ~CoreDataset();

  /**
   * @brief Adds a datatype to the core, loads the needed information from the
   * HDF5 file and creates the indices
   * @param datatype the datatype to be added
   * @param hdf5Io pointer to the object handling HDF5 io for this datatype
   */
  void addDatatype(
      const seerep_core_msgs::Datatype& datatype,
      std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io);

  /**
   * @brief Recreate the spatial rtree
   *
   * @param datatype the datatype for which to recreate the spatial rtree
   * @param hdf5Io pointer to the object handling HDF5 io for this datatype
   */
  void recreateSpatialRt(
      const seerep_core_msgs::Datatype& datatype,
      std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io);

  /**
   * @brief Returns a vector of UUIDs of datasets that match the query
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of datasets matching the query
   */
  std::vector<boost::uuids::uuid> getData(const seerep_core_msgs::Query& query);

  /**
   * @brief Returns a vector of UUIDs of instances that match the query
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of instances matching the query
   */
  std::vector<boost::uuids::uuid>
  getInstances(const seerep_core_msgs::Query& query);

  /**
   * @brief Adds an image to the spatial, temporal and semantic indices
   * @param dataset contains the relevant information for indexing
   */
  void addDataset(const seerep_core_msgs::DatasetIndexable& dataset);
  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the datatype to consider
   * @param labelPerCategory map from category to a vector of labels to be added
   * to the dataset
   * @param msgUuid the UUID of the targeted dataset
   */
  void addLabels(
      const seerep_core_msgs::Datatype& datatype,
      const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
          labelPerCategory,
      const boost::uuids::uuid& msgUuid);
  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the datatype to consider
   * @param datatypeSpecifics the datatypeSpecifics
   * @param labelPerCategory map from category to a vector of labels to be added
   * to the dataset
   * @param msgUuid the UUID of the targeted dataset
   */
  void addLabels(
      const seerep_core_msgs::Datatype& datatype,
      std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics>
          datatypeSpecifics,
      const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
          labelPerCategory,
      const boost::uuids::uuid& msgUuid);

  /**
   * @brief Get the minimum and maximum time interval for a dataset
   * @param datatypes A vector of datatypes for which the time bound has to be computed
   * @return seerep_core_msgs::AabbTime
   */
  seerep_core_msgs::AabbTime
  getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the minimum and maximum spatial bound for a dataset
   * @param datatypes A vector of datatypes for which the spatial bound has to
   * be computed
   * @return seerep_core_msgs::AABB
   */
  seerep_core_msgs::AABB
  getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the all categories saved in a project
   *
   * @param datatypes A vector of datatypes for which the categories have to be fetched
   * @return std::vector<std::string> vector of categories
   */
  std::unordered_set<std::string>
  getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the all labels saved in a project
   *
   * @param datatypes datatypes across which this is determined
   * @param category the category across which all labels have to be aggregated
   * @return std::vector<std::string> vector of labels
   */
  std::unordered_set<std::string>
  getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
               std::string category);

private:
  /**
   * @brief fills the member variables based on the HDF5 file
   * @param datatype the datatype to consider
   * @param hdf5io the HDF5io object handling the HDF5 io
   */
  void recreateDatasets(
      const seerep_core_msgs::Datatype& datatype,
      std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io);
  /**
   * @brief adds the dataset to the spatio-temporal-semantic indices
   * @param datatype the datatype to consider
   * @param dataset the information needed to add it to the indices
   */
  void addDatasetToIndices(const seerep_core_msgs::Datatype& datatype,
                           const seerep_core_msgs::DatasetIndexable& dataset);

  /**
   * @brief tries to add the dataset which couldn't be added to the spatial
   * index due to a missing transformtion into the frame of the index. If the
   * transformation is now available the data is added to the spatial index
   * @param datatype the datatype to consider
   */
  void tryAddingDataWithMissingTF(const seerep_core_msgs::Datatype& datatype);

  /**
   * @brief Checks the canTransform on the tf buffer for the indexable
   *
   * @param indexable the abstract indexable
   * @return true when a transform for the indexable dataset is in the tf buffer
   * @return false else
   */
  bool
  isSpatiallyTransformable(const seerep_core_msgs::DatasetIndexable& indexable);

  /**
   * @brief Get the Sensor Position As A A B B object
   *
   * @param indexable
   * @return seerep_core_msgs::AABB
   */
  seerep_core_msgs::AABB
  getSensorPositionAsAABB(const seerep_core_msgs::DatasetIndexable& indexable);

  /**
   * @brief Check if the created CGAL polygon follows the requirements. It
   * should be simple (no more than two vertices on an edge), convex (no
   * inward egdes), the vertices should be in a counter clockwise order.
   *
   * @param polygon_cgal a polygon defined with CGAL
   * @return true The polygon abides by CGAL requirements
   * @return false The polygon does not abide by CGAL requirements
   */
  bool verifyPolygonIntegrity(CGAL::Polygon_2<ExactKernel>& polygon_cgal);

  /**
   * @brief transforms the bounding box to the datasets frameId (mostly the map
   * frame)
   *
   * @param indexable the indexable to transform
   * @return The transformed AABB
   */
  seerep_core_msgs::AABB
  transformIndexableAABB(const seerep_core_msgs::DatasetIndexable& indexable);

  /**
   * @brief convert core msg polygon to CGAL polygon
   *
   * @param polygon core msg polygon
   * @return CGAL::Polygon_2<Kernel> cgal polygon
   */
  CGAL::Polygon_2<ExactKernel>
  toCGALPolygon(const seerep_core_msgs::Polygon2D& polygon);

  /**
   * @brief convert core msg aabb to CGAL aabb
   *
   * @param polygon core msg aabb
   * @return CGAL::Polygon_2<Kernel> cgal aabb
   */
  CGAL::Polygon_2<ExactKernel> toCGALPolygon(const seerep_core_msgs::AABB& aabb);

  /**
   * @brief determine if the axis aligned bounding box is fully or paritally
   * inside the oriented bounding box
   *
   * @param aabb axis aligned bounding box
   * @param polygon polygon
   * @param fullEncapsulation boolean variable to denote if the aabb fully
   * inside the obb
   * @param partialEncapsulation boolean variable to denote if the aabb
   * partially inside the obb
   */
  void intersectionDegree(const seerep_core_msgs::AABB& aabb,
                          const seerep_core_msgs::Polygon2D& polygon,
                          bool& fullEncapsulation, bool& partialEncapsulation);

  /**
   * @brief check whether cgal1 is encapsulated by cgal2
   *
   * @param cgal1 the (possibly) encapsulated polygon
   * @param cgal2 the (possibly) encapsulating polygon
   * @param z_partially a flag to indicate partial height encapsulation by cgal2
   * @param checkIfFullyEncapsulated whether to check fully encapsulation
   * @param fullEncapsulation is set to false when fullEncapsulation is not
   * valid. This flag is only set, when checkIfFullyEncapsulated is true
   * @param partialEncapsulation is set to true, when cgal1 is partially
   * encapsulated by cgal2
   */
  void intersectionDegreeCgalPolygons(CGAL::Polygon_2<ExactKernel> cgal1,
                                      CGAL::Polygon_2<ExactKernel> cgal2,
                                      bool z_partially,
                                      bool checkIfFullyEncapsulated,
                                      bool& fullEncapsulation,
                                      bool& partialEncapsulation);

  /**
   * @brief check if and to what degree AABB is encapsulated by polygon
   *
   * @param aabb the aabb to perform the check on
   * @param polygon the query polygon
   * @param aabb_cgal the core msgs aabb converted to a CGAL polygon
   * @param polygon_cgal the core msgs polygon converted to a CGAL polygon
   * @param fullEncapsulation whether aabb is fully encapsulated by the polygon
   * @param partialEncapsulation whether the aabb and polygon intersect on a
   *    subset of them in some way
   */
  void
  intersectionDegreeAABBinPolygon(const seerep_core_msgs::AABB& aabb,
                                  const seerep_core_msgs::Polygon2D& polygon,
                                  CGAL::Polygon_2<ExactKernel> aabb_cgal,
                                  CGAL::Polygon_2<ExactKernel> polygon_cgal,
                                  bool& fullEncapsulation,
                                  bool& partialEncapsulation);

  /**
   * @brief checks whether the enclosedMesh is really enclosed by the
   * enclosingPolygon
   *
   * Both the enclosedMesh and enclosingPolygon have to be convex objects
   *
   * @param enclosedMesh mesh to check whether it is enclosed by the polygon
   * @param enclosingPolygon polygon which is enclosing object
   * @param fullEncapsulation flag which is set when enclosedMesh is fully
   * encapsulated by the enclosingPolygon
   * @param partialEncapsulation flag which is set when enclosedMesh is
   * partially encapsulated by the enclosingPolygon
   */
  void checkIntersectionWithZExtrudedPolygon(
      CGSurfaceMesh enclosedMesh,
      const seerep_core_msgs::Polygon2D& enclosingPolygon,
      bool& fullEncapsulation, bool& partialEncapsulation);

  /**
   * @brief Creates a 2DPolygon by removing the 3rd dimension from the points of the Mesh
   *
   * @param mesh the input mesh
   *
   * @return the reduced mesh as a CGAL 2D polygon
   */
  CGPolygon_2 reduceZDimension(const CGSurfaceMesh& mesh);

  /**
   * @brief creates a SurfaceMesh from a seerep polygon 2D
   *
   * @param seerep_polygon the polygon to create the mesh from
   *
   * @return the resulting SurfaceMesh
   */
  CGSurfaceMesh toSurfaceMesh(const seerep_core_msgs::Polygon2D& seerep_polygon);

  void getUuidsFromMap(
      std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                         boost::hash<boost::uuids::uuid>>& datasetInstancesMap,
      std::vector<boost::uuids::uuid>& datasets,
      std::vector<boost::uuids::uuid>& result);

  /**
   * @brief Convert polygon to a smallest possible encapsulating AABB
   *
   * @param polygon core msg polygon
   * @return seerep_core_msg::AABB core msg aabb
   */
  seerep_core_msgs::AABB
  polygonToAABB(const seerep_core_msgs::Polygon2D& polygon);

  /**
   * @brief queries the spatial index and returns a vector of bounding box / UUID
   * pairs matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return vector of bounding box / UUID pairs matching the query
   */
  std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
  querySpatial(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
               const seerep_core_msgs::Query& query);

  /**
   * @brief queries the spatial index of the sensor position and returns a vector
   * of bounding box / UUID pairs matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return vector of bounding box / UUID pairs matching the query
   */
  std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
  querySpatialSensorPos(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                        const seerep_core_msgs::Query& query);

  /**
   * @brief queries the rtree
   *
   * @param rt
   * @param polygon
   * @param coreDatatype
   * @param queryFullyEncapsulated
   * @return std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
   */
  std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
  queryRtree(const seerep_core_msgs::rtree& rt,
             seerep_hdf5_core::Hdf5CoreDatatypeInterface& coreDatatype,
             const seerep_core_msgs::Polygon2D& polygon,
             const bool queryFullyEncapsulated);

  /**
   * @brief queries the temporal index and returns a vector of temporal bounding
   * box / UUID pairs matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return vector of temporal bounding box / UUID pairs matching the query
   */
  std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>
  queryTemporal(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                const seerep_core_msgs::Query& query);
  /**
   * @brief queries the semantic index and returns the UUIDs of the images
   * matching the query
   * @param datatypeSpecifics the datatype specific information to be used in
   * the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::optional<std::set<boost::uuids::uuid>>
  querySemantic(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                const seerep_core_msgs::Query& query);

  /**
   * @brief queries the semantic index where the dataset contains any of the labels
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::set<boost::uuids::uuid> querySemanticWithAnyOfLabels(
      std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
      const seerep_core_msgs::Query& query);

  /**
   * @brief queries the semantic index where the dataset contains all of the labels
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::set<boost::uuids::uuid> querySemanticWithAllTheLabels(
      std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
      const seerep_core_msgs::Query& query);

  /**
   * @brief intersects the results of the spatial, temporal and semantic query
   * and returns the UUIDs of the images matching the query in all three modalities
   * @param rt_result the result of the spatial query
   * @param rt_resultSensorPos the result of the spatial query of the sensor position
   * @param timetree_result the result of the temporal query
   * @param semanticResult the result of the semantic query
   * @param instanceResult the result of the instance based query
   * @param dataUuids the uuids of the dataset specified in the query
   * @return vector of UUIDs of the images matching the query in all three modalities
   */
  std::vector<boost::uuids::uuid> intersectQueryResults(
      std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_result,
      std::optional<std::vector<seerep_core_msgs::AabbIdPair>>&
          rt_resultSensorPos,
      std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>&
          timetree_result,
      std::optional<std::set<boost::uuids::uuid>>& semanticResult,
      std::optional<std::vector<boost::uuids::uuid>>& instanceResult,
      const std::optional<std::vector<boost::uuids::uuid>>& dataUuids,
      bool sortByTime = false);
  /**
   * @brief sorts the uuids of the result set based on their timestamp
   *
   * @param timetree_result the result from the timetree query. Contains uuids
   * and the corresponding timestamp
   * @param intersectedResult the intersected uuids of all parts of the query
   * @return std::vector<boost::uuids::uuid> uuids of the result set sorted by timestamp
   */
  std::vector<boost::uuids::uuid> sortResultByTime(
      std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>&
          timetree_result,
      std::optional<std::set<boost::uuids::uuid>> intersectionResult =
          std::nullopt);

  /**
   * @brief intersects a vector of sets pairwise recursively until one
   * intersection set remains
   * @param vectorOfSets the vector of sets to be intersected
   * @return set of UUIDs of the intersection result
   */
  std::set<boost::uuids::uuid>
  intersectVectorOfSets(std::vector<std::set<boost::uuids::uuid>>& vectorOfSets);

  /**
   * @brief return the UUIDs of all stored datasets. Uses the timeTree, because
   * all datasets are in there
   * @param datatypeSpecifics the datatype specifics of the targeted data type
   * @param sortByTime flag if the result set should be sorted by the timestamp
   * of the data
   * @return vector of UUIDs of all data sets
   */
  std::vector<boost::uuids::uuid>
  getAllDatasetUuids(std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics>
                         datatypeSpecifics,
                     bool sortByTime);

  /** @brief the frame id of the spatial index*/
  std::string m_frameId;
  /** @brief shared pointer to the object handling transformations */
  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  /** @brief shared pointer to the object handling the instances */
  std::shared_ptr<seerep_core::CoreInstances> m_coreInstances;
  /** @brief map from the datatype to the struct with the specific objects for that datatype*/
  std::unordered_map<seerep_core_msgs::Datatype,
                     std::shared_ptr<DatatypeSpecifics>>
      m_datatypeDatatypeSpecificsMap;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_DATASET_H_
