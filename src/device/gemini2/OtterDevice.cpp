// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "OtterDevice.hpp"
#include "G2PropertyAccessors.hpp"
#include "OtterY14Unpacker.hpp"
#include "FilterDecorator.hpp"
#include "InternalTypes.hpp"
#include "utils/PublicTypeHelper.hpp"

#include "sensor/video/DisparityBasedSensor.hpp"
#include "property/CommonPropertyAccessors.hpp"
#include "property/PropertyServer.hpp"
#include "property/VendorPropertyAccessor.hpp"
#include "timestamp/GlobalTimestampFitter.hpp"
#include "timestamp/FrameTimestampCalculator.hpp"

namespace libobsensor {

// Depth unit of the device's disparity to depth output, in mm, measured in both Default work modes
constexpr float OTTER_DEPTH_UNIT_MM = 0.4f;

OtterDepthWorkModeManager::OtterDepthWorkModeManager(IDevice *owner) : G2DepthWorkModeManager(owner) {}

void OtterDepthWorkModeManager::writeDepthWorkMode(const OBDepthWorkMode_Internal &mode) {
    auto propServer = getOwner()->getPropertyServer();
    propServer->setStructureDataT<OBDepthWorkMode_Internal>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, mode);
}

OtterDevice::OtterDevice(const std::shared_ptr<const IDeviceEnumInfo> &info) : G2Device(info) {
    // G2Device's constructor ran G2Device::init(), as virtual calls do not reach this class during construction
    replaceG2Components();
}

OtterDevice::~OtterDevice() noexcept {}

void OtterDevice::init() {
    G2Device::init();
    replaceG2Components();
}

void OtterDevice::replaceG2Components() {
    // The Gemini2 work mode manager's v1.1 write crashes the Otter's firmware
    deregisterComponent(OB_DEV_COMPONENT_DEPTH_WORK_MODE_MANAGER);
    registerComponent(OB_DEV_COMPONENT_DEPTH_WORK_MODE_MANAGER, std::make_shared<OtterDepthWorkModeManager>(this));

    initDepthProperties();
    initDepthSensor();

    // The disparity to depth conversion processes the depth channel, which carries the right IR image in the binocular modes
    getPropertyServer()->setPropertyValueT<bool>(OB_PROP_DISPARITY_TO_DEPTH_BOOL, !isBinocularMode());
}

bool OtterDevice::isBinocularMode() {
    auto manager = getComponentT<G2DepthWorkModeManager>(OB_DEV_COMPONENT_DEPTH_WORK_MODE_MANAGER);
    return G2DepthWorkModeManager::isRightIrFromDepthChannel(manager->getCurrentDepthWorkMode().mode.optionCode);
}

void OtterDevice::initDepthProperties() {
    auto propertyServer = getPropertyServer();

    auto d2dPropertyAccessor = std::make_shared<OtterDisp2DepthPropertyAccessor>(this);
    propertyServer->registerProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, "rw", "rw", d2dPropertyAccessor);
    propertyServer->registerProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, "rw", "rw", d2dPropertyAccessor);
    propertyServer->registerProperty(OB_STRUCT_DEPTH_PRECISION_SUPPORT_LIST, "r", "r", d2dPropertyAccessor);

    // Same command channel as the laser: the depth interface, which carries the right IR stream in the binocular modes
    const auto commandSensor = isSensorExists(OB_SENSOR_DEPTH) ? OB_SENSOR_DEPTH : OB_SENSOR_IR_RIGHT;
    if(isSensorExists(commandSensor)) {
        auto portInfo       = getSensorPortInfo(commandSensor);
        auto vendorAccessor = std::make_shared<LazySuperPropertyAccessor>(
            [this, portInfo]() { return std::make_shared<VendorPropertyAccessor>(this, getSourcePort(portInfo)); });
        propertyServer->registerProperty(OB_PROP_IR_LED_MASK_BOOL, "rw", "rw", vendorAccessor);
        propertyServer->registerProperty(OB_PROP_LIGHT_RGB_WHITE_BOOL, "rw", "rw", vendorAccessor);
        propertyServer->registerProperty(OB_PROP_LIGHT_IR_WHITE_BOOL, "rw", "rw", vendorAccessor);
        propertyServer->registerProperty(OB_PROP_LIGHT_RGB_WHITE_ELECTRIC_INT, "rw", "rw", vendorAccessor);
        propertyServer->registerProperty(OB_PROP_LIGHT_IR_WHITE_ELECTRIC_INT, "rw", "rw", vendorAccessor);
    }

    // Not supported on this device
    propertyServer->unregisterProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL);
    propertyServer->unregisterProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL);
    propertyServer->unregisterProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_DIFF_INT);
    propertyServer->unregisterProperty(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_SPECKLE_SIZE_INT);
}

void OtterDevice::initDepthSensor() {
    // In the binocular work modes the depth channel carries the right IR stream and there is no depth sensor to replace
    if(!isSensorExists(OB_SENSOR_DEPTH)) {
        return;
    }
    auto depthPortInfo = getSensorPortInfo(OB_SENSOR_DEPTH);

    deregisterComponent(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
    deregisterComponent(OB_DEV_COMPONENT_DEPTH_SENSOR);
    registerComponent(
        OB_DEV_COMPONENT_DEPTH_SENSOR,
        [this, depthPortInfo]() {
            auto port   = getSourcePort(depthPortInfo);
            auto sensor = std::make_shared<DisparityBasedSensor>(this, OB_SENSOR_DEPTH, port);

            std::vector<FormatFilterConfig> formatFilterConfigs = {
                { FormatFilterPolicy::REPLACE, OB_FORMAT_MJPG, OB_FORMAT_RLE, nullptr },
                { FormatFilterPolicy::REPLACE, OB_FORMAT_Y14, OB_FORMAT_Y16, std::make_shared<FilterDecorator>("OtterY14Unpacker", std::make_shared<OtterY14Unpacker>()) },
            };
            sensor->updateFormatFilterConfig(formatFilterConfigs);

            auto frameTimestampCalculator = videoFrameTimestampCalculatorCreator_();
            sensor->setFrameTimestampCalculator(frameTimestampCalculator);

            auto globalFrameTimestampCalculator = std::make_shared<GlobalTimestampCalculator>(this, deviceTimeFreq_, frameTimeFreq_);
            sensor->setGlobalTimestampCalculator(globalFrameTimestampCalculator);

            // Depth comes from the device's disparity to depth conversion
            auto propServer = getPropertyServer();
            propServer->setPropertyValueT<bool>(OB_PROP_DISPARITY_TO_DEPTH_BOOL, true);
            sensor->markOutputDisparityFrame(false);

            sensor->setDepthUnit(OTTER_DEPTH_UNIT_MM);

            initSensorStreamProfile(sensor);

            return sensor;
        },
        true);
}

}  // namespace libobsensor
