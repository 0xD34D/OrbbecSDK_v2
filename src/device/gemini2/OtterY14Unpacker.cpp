// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "OtterY14Unpacker.hpp"
#include "frame/FrameFactory.hpp"
#include "stream/StreamProfile.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

void OtterY14Unpacker::updateConfig(std::vector<std::string> &params) {
    utils::unusedVar(params);
}

void OtterY14Unpacker::setConfigData(void *data, uint32_t size) {
    utils::unusedVar(data);
    utils::unusedVar(size);
}

const std::string &OtterY14Unpacker::getConfigSchema() const {
    static const std::string schema;
    return schema;
}

void OtterY14Unpacker::reset() {
    srcStreamProfile_.reset();
    dstStreamProfile_.reset();
}

std::shared_ptr<Frame> OtterY14Unpacker::process(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto         videoFrame = frame->as<VideoFrame>();
    const size_t pixelCount = static_cast<size_t>(videoFrame->getWidth()) * videoFrame->getHeight();
    const size_t groupCount = pixelCount / 4;
    if(pixelCount % 4 != 0 || frame->getDataSize() < groupCount * 7) {
        LOG_WARN_INTVL("Drop Y14 frame: unexpected data size {} for {}x{}", frame->getDataSize(), videoFrame->getWidth(), videoFrame->getHeight());
        return nullptr;
    }

    auto streamProfile = frame->getStreamProfile();
    if(!srcStreamProfile_ || srcStreamProfile_.get() != streamProfile.get()) {
        srcStreamProfile_ = streamProfile;
        dstStreamProfile_ = streamProfile->clone();
        dstStreamProfile_->setFormat(OB_FORMAT_Y16);
    }

    auto outFrame = FrameFactory::createFrameFromStreamProfile(dstStreamProfile_);
    if(!outFrame) {
        LOG_ERROR_INTVL("Create frame by frame factory failed!");
        return nullptr;
    }
    outFrame->copyInfoFromOther(frame);

    const uint8_t *src = frame->getData();
    auto          *dst = reinterpret_cast<uint16_t *>(outFrame->getDataMutable());
    for(size_t i = 0; i < groupCount; i++, src += 7, dst += 4) {
        uint64_t bits = 0;
        for(int b = 0; b < 7; b++) {
            bits = (bits << 8) | src[b];
        }
        dst[0] = static_cast<uint16_t>((bits >> 42) & 0x3FFF);
        dst[1] = static_cast<uint16_t>((bits >> 28) & 0x3FFF);
        dst[2] = static_cast<uint16_t>((bits >> 14) & 0x3FFF);
        dst[3] = static_cast<uint16_t>(bits & 0x3FFF);
    }
    return outFrame;
}

}  // namespace libobsensor
