#ifndef ADA_LO_CLOUD_CONVERT_INTERFACE_HPP_
#define ADA_LO_CLOUD_CONVERT_INTERFACE_HPP_

#include "common/eigen_types.h"

namespace zjloc
{
     class CloudConvertInterface
     {
     public:
          CloudConvertInterface() {}

          virtual ~CloudConvertInterface() {}

          CloudConvertInterface(const CloudConvertInterface &) = delete;

          CloudConvertInterface &operator=(const CloudConvertInterface &) = delete;

          virtual void AddData(Eigen::Vector3d in) = 0;
     };
}

#endif // ADA_LO_CLOUD_CONVERT_INTERFACE_HPP_