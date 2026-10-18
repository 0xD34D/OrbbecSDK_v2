// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#pragma once
#include "IFilter.hpp"

#include <memory>
#include <string>
#include <vector>

namespace libobsensor {

class StreamProfile;

/**
 * @brief Unpacks the CR-Scan Otter's Y14 depth stream to Y16.
 *
 * Every 4 pixels occupy 7 bytes, packed MSB-first: the first pixel is the top 14 bits of the 56-bit big-endian group.
 */
class OtterY14Unpacker : public IFilterBase {
public:
    OtterY14Unpacker()                    = default;
    virtual ~OtterY14Unpacker() noexcept override = default;

    void                   updateConfig(std::vector<std::string> &params) override;
    void                   setConfigData(void *data, uint32_t size) override;
    const std::string     &getConfigSchema() const override;
    void                   reset() override;
    std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) override;

private:
    std::shared_ptr<const StreamProfile> srcStreamProfile_;
    std::shared_ptr<StreamProfile>       dstStreamProfile_;
};

}  // namespace libobsensor
