// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once
#include "G2Device.hpp"
#include "G2DepthWorkModeManager.hpp"

namespace libobsensor {

/**
 * @brief Depth work mode manager for the CR-Scan Otter, which takes the work mode as a classic structure write.
 */
class OtterDepthWorkModeManager : public G2DepthWorkModeManager {
public:
    explicit OtterDepthWorkModeManager(IDevice *owner);
    virtual ~OtterDepthWorkModeManager() noexcept override = default;

protected:
    void writeDepthWorkMode(const OBDepthWorkMode_Internal &mode) override;
};

/**
 * @brief Creality CR-Scan Otter: a Gemini2 (MX6600) platform device whose depth is converted from disparity on the device.
 */
class OtterDevice : public G2Device {
public:
    OtterDevice(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~OtterDevice() noexcept override;

    // Also runs on reset(), e.g. after a depth work mode switch
    void init() override;

private:
    void replaceG2Components();
    void initDepthSensor();
    void initDepthProperties();
};

}  // namespace libobsensor
