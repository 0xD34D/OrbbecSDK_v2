// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "HidDevicePort.hpp"

#include "frame/FrameFactory.hpp"
#include "logger/Logger.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/Utils.hpp"
#include "usb/enumerator/UsbEnumeratorLibusb.hpp"

namespace libobsensor {
HidDevicePort::HidDevicePort(const std::shared_ptr<IUsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : portInfo_(portInfo), usbDevice_(usbDevice), isStreaming_(false), frameQueue_(10) {

    auto libusbDevice = std::dynamic_pointer_cast<UsbDeviceLibusb>(usbDevice_);
    auto epDesc       = libusbDevice->getEndpointDesc(portInfo->infIndex, LIBUSB_ENDPOINT_TRANSFER_TYPE_INTERRUPT, LIBUSB_ENDPOINT_IN);
    endpointAddress_  = epDesc.bEndpointAddress;
    maxPacketSize_    = epDesc.wMaxPacketSize;

    LOG_DEBUG("HidDevicePort::HidDevicePort done");
}

HidDevicePort::~HidDevicePort() noexcept {
    if(isStreaming_) {
        frameQueue_.stop();
        TRY_EXECUTE(stopStream());
    }
    LOG_DEBUG("HidDevicePort::~HidDevicePort done");
}

// The interface is held only while streaming, so other users of it (such as the kernel's HID driver) get it back in between
void HidDevicePort::claimInterface() {
    auto libusbDevice    = std::dynamic_pointer_cast<UsbDeviceLibusb>(usbDevice_);
    auto libusbDevHandle = libusbDevice->getLibusbDeviceHandle();
    if(libusb_kernel_driver_active(libusbDevHandle, portInfo_->infIndex) == 1) {
        auto res = libusb_detach_kernel_driver(libusbDevHandle, portInfo_->infIndex);
        if(res != LIBUSB_SUCCESS) {
            THROW_IO_EXCEPTION("detach kernel driver failed, error: " + std::string(libusb_strerror(res)));
        }
        kernelDriverDetached_ = true;
    }
    auto res = libusb_claim_interface(libusbDevHandle, portInfo_->infIndex);
    if(res == LIBUSB_ERROR_BUSY) {
        LOG_WARN("libusb_claim_interface busy, retrying once. Interface: {}", portInfo_->infIndex);
        res = libusb_claim_interface(libusbDevHandle, portInfo_->infIndex);
    }
    if(res != LIBUSB_SUCCESS) {
        releaseInterface();
        THROW_IO_EXCEPTION("claim interface failed, error: " + std::string(libusb_strerror(res)));
    }
    interfaceClaimed_ = true;
}

void HidDevicePort::releaseInterface() {
    auto libusbDevice    = std::dynamic_pointer_cast<UsbDeviceLibusb>(usbDevice_);
    auto libusbDevHandle = libusbDevice->getLibusbDeviceHandle();
    if(interfaceClaimed_) {
        auto res = libusb_release_interface(libusbDevHandle, portInfo_->infIndex);
        if(res != LIBUSB_SUCCESS) {
            LOG_WARN("release interface failed, error: {}", libusb_strerror(res));
        }
        interfaceClaimed_ = false;
    }
    if(kernelDriverDetached_) {
        auto res = libusb_attach_kernel_driver(libusbDevHandle, portInfo_->infIndex);
        if(res != LIBUSB_SUCCESS) {
            LOG_WARN("attach kernel driver failed, error: {}", libusb_strerror(res));
        }
        kernelDriverDetached_ = false;
    }
}

void HidDevicePort::startStream(MutableFrameCallback callback) {
    if(isStreaming_) {
        THROW_WRONG_API_CALL_SEQUENCE_EXCEPTION("HidDevicePort::startStream() called while streaming");
    }
    claimInterface();
    isStreaming_ = true;
    frameQueue_.start(callback);

    streamThread_ = std::thread([this]() {
        auto libusbDevice    = std::dynamic_pointer_cast<UsbDeviceLibusb>(usbDevice_);
        auto libusbDevHandle = libusbDevice->getLibusbDeviceHandle();
        while(isStreaming_) {
            auto frame = FrameFactory::createFrame(OB_FRAME_UNKNOWN, OB_FORMAT_UNKNOWN, maxPacketSize_);
            int  transferred;
            auto res = libusb_interrupt_transfer(libusbDevHandle, endpointAddress_, frame->getDataMutable(), static_cast<int>(frame->getDataSize()),
                                                 &transferred, 1000);
            if(res != LIBUSB_SUCCESS && isStreaming_) {
                LOG_WARN_INTVL(utils::string::to_string() << "interrupt transfer failed, error: " << libusb_strerror(res));
                continue;
            }
            const auto timestamp = utils::getHostTimestampUs();
            frame->setSystemTimeStampUsec(timestamp.systemTimeUs);
            frame->setSteadyTimeStampUsec(timestamp.steadyTimeUs);
            frameQueue_.enqueue(frame);
        }
    });
    LOG_DEBUG("HidDevicePort::startStream done");
}

void HidDevicePort::stopStream() {
    if(!isStreaming_) {
        THROW_WRONG_API_CALL_SEQUENCE_EXCEPTION("HidDevicePort::stopStream() called while not streaming");
    }
    isStreaming_ = false;

    streamThread_.join();
    frameQueue_.flush();
    releaseInterface();

    LOG_DEBUG("HidDevicePort::stopStream done");
}

std::shared_ptr<const SourcePortInfo> HidDevicePort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace libobsensor
