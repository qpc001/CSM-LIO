/**
 * Copyright 2023 WANG Guanhua (wangguanhua999@gmail.com)
*/

#ifndef CSMLIO_MAPPING_CSM_LIO_INTERFACE_H_
#define CSMLIO_MAPPING_CSM_LIO_INTERFACE_H_

#include <string>
#include "csmlio/sensor/timed_point_cloud_data.h"
#include "csmlio/sensor/imu_data.h"
#include "csmlio/sensor/odometry_data.h"

namespace csmlio {
namespace mapping {

class CSMLioInterface {
  public:

    CSMLioInterface() {}
    ~CSMLioInterface() {}

    CSMLioInterface(const CSMLioInterface&) = delete;
    CSMLioInterface& operator=(const CSMLioInterface&) = delete;

    /// 定义纯虚函数作为接口，专为处理Dispatchable类中的数据传递；
    /// Dispatchable类include此虚基类（而非衍生类）能够避免交叉包含。
    virtual void ProcessSensorData(
        const std::string& sensor_id,
        const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
    virtual void ProcessSensorData(const std::string& sensor_id,
                            const sensor::ImuData& imu_data) = 0;
    virtual void ProcessSensorData(const std::string& sensor_id,
                            const sensor::OdometryData& odometry_data) = 0;

  private:

};

} // namespace mapping
} // namespace csmlio





#endif // CSMLIO_MAPPING_CSM_LIO_INTERFACE_H_