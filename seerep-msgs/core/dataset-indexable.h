#ifndef SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
#define SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_

#include <functional>

#include "aabb.h"
#include "header.h"

namespace seerep_core_msgs
{
struct DatasetIndexable
{
  Header header;
  AABB boundingbox;
  std::vector<std::string> labels;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
