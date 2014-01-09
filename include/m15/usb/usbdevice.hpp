#ifndef M15_USBDEVICE_HPP
#define M15_USBDEVICE_HPP

#include "usbdevicecoredriver.hpp"

#include <algorithm>
#include <utility>

namespace m15
{

class UsbEventListener
{
public:
    virtual void onConnect() {}
    virtual void onDisconnect() {}
    virtual void onEnumerationComplete(Usb::Speed speed) {}
    virtual void onConfigure() {}
    virtual void onReset() {}
    virtual void onResume() {}
    virtual void onSuspend() {}
};

class UsbDescriptorProvider
{
public:
    virtual std::pair<const void*, std::uint32_t> deviceDescriptor(
            Usb::Speed enumeratedSpeed) = 0;
    virtual std::pair<const void*, std::uint32_t> configurationDescriptor(
            Usb::Speed enumeratedSpeed, std::uint8_t index) = 0;
    virtual std::pair<const void*, std::uint32_t> stringDescriptor(
            Usb::Speed enumeratedSpeed, std::uint8_t index) = 0;

    virtual std::pair<const void*, std::uint32_t> deviceQualifierDescriptor(
            Usb::Speed enumeratedSpeed)
    {
        return std::pair<const void*, std::uint32_t>(NULL, 0);
    }

    virtual std::pair<const void*, std::uint32_t> otherSpeedConfigurationDescriptor(
            Usb::Speed enumeratedSpeed, std::uint8_t index)
    {
        return std::pair<const void*, std::uint32_t>(NULL, 0);
    }
};

template <typename TUsbSelector>
struct default_usb_device_options
        : public detail::default_usb_device_core_driver_options<TUsbSelector>
{
    //! If this flag is set, the USB device reports to the host that it is
    //! self-powered. Otherwise, it signals that it is bus-powered.
    static const bool self_powered = true;
};

//! The USB device driver.
//!
//! Handling a SETUP packet
//! The derived class has to implement a method with the signature
//! <tt>void classStandardSetupPacketHandler(const Usb::SetupPacket& packet)</tt>.
//!
template <typename TUsbSelector,
          typename TDerived,
          typename TOptions = default_usb_device_options<TUsbSelector> >
class UsbDevice
        : public detail::UsbDeviceCoreDriver<
                     TUsbSelector,
                     UsbDevice<TUsbSelector, TDerived, TOptions>,
                     TOptions>
                     //! \todo Inherit protected?
{
    typedef detail::UsbDeviceCoreDriver<
            TUsbSelector,
            UsbDevice<TUsbSelector, TDerived, TOptions>,
            TOptions> base_type;
public:
    typedef UsbDevice<TUsbSelector, TDerived, TOptions> self_type;

    //! An enumeration of possible device states as defined by the USB standard.
    enum State
    {
        PoweredState,
        DefaultState,
        AddressState,
        ConfiguredState,
        SuspendedState
    };

    explicit UsbDevice(UsbDescriptorProvider* descriptorProvider,
                       UsbEventListener* listener = 0)
        : m_remoteWakeupEnabled(false),
          m_state(PoweredState),
          m_preSuspendState(PoweredState),
          m_activeConfiguration(0),
          m_defaultControlPipeState(DcpIdle),
          m_descriptorProvider(descriptorProvider),
          m_listener(listener)
    {
    }

    //! Returns the event listener which has been attached to this object.
    UsbEventListener* listener() const
    {
        return m_listener;
    }

    //! Returns the state of the USB device.
    State state() const
    {
        return m_state;
    }

protected:
    //! Prepares the endpoint OUT 0 (which belongs to the Default Control Pipe)
    //! to receive \p size bytes of data which will be stored in the given
    //! \p buffer. This method must be called in response to a SETUP packet
    //! whose direction is OUT (host to device) in order to switch the
    //! Default Control Pipe to the DATA-OUT stage.
    //!
    //! If \p notifyClassDriver is set, the class-driver will be informed via
    //! classDefaultControlPipeDataReceivedHandler() when the transfer has
    //! been completed.
    //!
    //! \note This method must not be called, if the \p wLength field of the
    //! SETUP packet is zero. In this case, the Default Control Pipe has to
    //! enter the IN-directed STATUS stage instead, which is achieved with
    //! sendDefaultControlPipeStatus().
    void prepareDefaultControlPipeDataReception(
            void* buffer, std::uint32_t size, bool notifyClassDriver = false);
    //! Prepares the endpoint OUT 0 (which belongs to the Default Control Pipe)
    //! to receive a zero-length status packet. This method must be called to
    //! complete control transfers with a data stage consisting of IN packets.
    void prepareDefaultControlPipeStatusReception();
    //! Sends a zero-length packet on the endpoint IN 0 (which
    //! belongs to the Default Control Pipe). This method must be called to
    //! complete control transfers without a data stage or with a data stage
    //! consisting of OUT packets.
    //! Simultaneously, endpoint OUT 0 is prepared to receive a new SETUP
    //! packet.
    void sendDefaultControlPipeStatus();
    //! Sends \p size bytes from the given \p data via the endpoint IN 0
    //! (which belongs to the Default Control Pipe) to the host. This
    //! switches the Default Control Pipe to the DATA-IN stage in response
    //! to a SETUP packet with direction IN.
    //!
    //! If \p notifyClassDriver is set, the class-driver will be informed via
    //! classDefaultControlPipeDataSentHandler() when the transfer has
    //! been completed.
    void sendDefaultControlPipeData(
            const void* data, std::uint32_t size,
            bool notifyClassDriver = false);
    //! Stalls the default control pipe.
    //! Stalls the endpoints IN 0 and OUT 0, which make up the Default Control
    //! Pipe. The endpoint OUT 0 is prepared to receive a new SETUP packet.
    //! When a SETUP packet arrives, the stall condition is cleared
    //! automatically.
    void stallDefaultControlPipe();

private:
    //! An enumeration of states of the Default Control Pipe (endpoints IN 0
    //! and OUT 0).
    enum DefaultControlPipeState
    {
        DcpIdle,
        DcpSetupStage,
        DcpDataInStage,
        DcpSignallingDataInStage,
        DcpDataOutStage,
        DcpSignallingDataOutStage,
        DcpStatusInStage,
        DcpStatusOutStage
    };

    //! The size and remaining bytes of a transfer via the Default Control Pipe.
    struct DcpTransferData
    {
        std::uint32_t bufferSize;
        std::uint32_t numRemainingBytes;
    };

    //! If set, the remote wakeup feature is enabled.
    bool m_remoteWakeupEnabled;
    //! The state of the device.
    State m_state;
    //! The state in which the device has been just before a suspend event.
    State m_preSuspendState;
    //! The currently active configuration.
    std::uint8_t m_activeConfiguration;
    //! The state of the Default Control Pipe (endpoints IN 0 and OUT 0).
    DefaultControlPipeState m_defaultControlPipeState;
    //! A memory for up to three SETUP packets (the USB 2.0 specification allows
    //! sending up to three SETUP packets back-to-back).
    //! \todo Align this memory according to the core driver traits.
    std::uint64_t m_setupPacket[3];
    //! The object which supplies the USB descriptors.
    UsbDescriptorProvider* m_descriptorProvider;
    //! The listener which is informed about USB events.
    UsbEventListener* m_listener;

    DcpTransferData m_dcpInTransfer;
    DcpTransferData m_dcpOutTransfer;


    //! Analyses the SETUP \p packet which addresses the device and forwards it
    //! to one of the <tt>handleXDeviceRequest()</tt> handlers.
    void dispatchStandardDeviceRequests(const Usb::SetupPacket& packet);
    void handleClearFeatureDeviceRequest(const Usb::SetupPacket& packet);
    void handleGetConfigurationDeviceRequest(const Usb::SetupPacket& packet);
    void handleGetDescriptorDeviceRequest(const Usb::SetupPacket& packet);
    void handleGetStatusDeviceRequest(const Usb::SetupPacket& packet);
    void handleSetAddressDeviceRequest(const Usb::SetupPacket& packet);
    void handleSetConfigurationDeviceRequest(const Usb::SetupPacket& packet);
    void handleSetFeatureDeviceRequest(const Usb::SetupPacket& packet);

    void dispatchEndpointRequests(const Usb::SetupPacket& packet);

    //! Casts this object to the derived class.
    TDerived& derived()
    {
        return *static_cast<TDerived*>(this);
    }

    //! Casts this object to the derived class.
    const TDerived& derived() const
    {
        return *static_cast<const TDerived*>(this);
    }



    // ---- CRTP-"callbacks" invoked from the low-level USB core device driver -

    void deviceConnectHandler();
    void deviceDisconnectHandler();
    void deviceEnumerationCompletedHandler();
    void deviceIncompleteIsochronousInHandler();
    void deviceIncompleteIsochronousOutHandler();
    void deviceResumeHandler();
    void deviceStartOfFrameHandler();
    void deviceSuspendHandler();

    //! Called by the low-level core driver when the IN endpoint with the given
    //! \p endpointNumber has finished its data stage (i.e. the data has been
    //! sent to the host).
    void deviceInTransactionCompletedHandler(std::uint8_t endpointNumber,
                                             std::uint32_t numBytesSent);
    //! Called by the low-level core driver when the OUT endpoint with the given
    //! \p endpointNumber has finished its data stage (i.e. the data from the
    //! host has been received).
    void deviceOutTransactionCompletedHandler(std::uint8_t endpointNumber,
                                              std::uint32_t numBytesReceived);
    //! Called by the low-level core driver when one or more SETUP packets has
    //! been received via the endpoint with the given \p address. The number of
    //! SETUP packets is passed in \p numSetupPackets.
    void deviceSetupPacketHandler(std::uint8_t address,
                                  std::uint8_t numSetupPackets);
    //! Called by the low-level core driver upon a USB reset event. Enables the
    //! endpoints OUT 0 and IN 0.
    void deviceUsbResetHandler();

    friend base_type;
};

// ----=====================================================================----
//     Protected methods
// ----=====================================================================----

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::prepareDefaultControlPipeDataReception(
        void* buffer, std::uint32_t size, bool notifyClassDriver)
{
    m_dcpOutTransfer.bufferSize = size;
    m_dcpOutTransfer.numRemainingBytes = size;
    if (notifyClassDriver)
    {
        m_defaultControlPipeState = DcpSignallingDataOutStage;
    }
    else
    {
        m_defaultControlPipeState = DcpDataOutStage;
    }
    this->prepareReception(0x00, buffer, size);
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::prepareDefaultControlPipeStatusReception()
{
    m_defaultControlPipeState = DcpStatusOutStage;
    this->prepareReception(0x00, NULL, 0);
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::sendDefaultControlPipeData(
        const void* data, std::uint32_t size, bool notifyClassDriver)
{
    m_dcpInTransfer.bufferSize = size;
    m_dcpInTransfer.numRemainingBytes = size;
    if (notifyClassDriver)
    {
        m_defaultControlPipeState = DcpSignallingDataInStage;
    }
    else
    {
        m_defaultControlPipeState = DcpDataInStage;
    }
    this->sendData(0x80, data, size);
    // Prepare the endpoint OUT 0 for a new SETUP packet (just in case it
    // arrives while we are still sending data).
    this->prepareSetupPacketReception(&m_setupPacket, sizeof(m_setupPacket));
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::sendDefaultControlPipeStatus()
{
    m_defaultControlPipeState = DcpStatusInStage;
    // Send a zero-length packet on the endpoint IN 0.
    this->sendData(0x80, 0, 0);
    // Prepare the endpoint OUT 0 for a new SETUP packet (just in case it
    // arrives while we are still sending data).
    this->prepareSetupPacketReception(&m_setupPacket, sizeof(m_setupPacket));
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::stallDefaultControlPipe()
{
    // Stall the endpoints IN 0 and OUT 0.
    m_defaultControlPipeState = DcpIdle;
    this->setEndpointStall(0x80);
    this->setEndpointStall(0x00);
    // Prepare the endpoint OUT 0 to receive a new SETUP packet.
    this->prepareSetupPacketReception(&m_setupPacket, sizeof(m_setupPacket));
}

// ----=====================================================================----
//     Private methods
// ----=====================================================================----

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::dispatchStandardDeviceRequests(
        const Usb::SetupPacket& packet)
{
    switch (packet.bRequest)
    {
        case Usb::SetupPacket::GetStatus:
            handleGetStatusDeviceRequest(packet);
            break;

        case Usb::SetupPacket::ClearFeature:
            handleClearFeatureDeviceRequest(packet);
            break;

        case Usb::SetupPacket::SetFeature:
            handleSetFeatureDeviceRequest(packet);
            break;

        case Usb::SetupPacket::SetAddress:
            handleSetAddressDeviceRequest(packet);
            break;

        case Usb::SetupPacket::GetDescriptor:
            handleGetDescriptorDeviceRequest(packet);
            break;

        case Usb::SetupPacket::GetConfiguration:
            handleGetConfigurationDeviceRequest(packet);
            break;

        case Usb::SetupPacket::SetConfiguration:
            handleSetConfigurationDeviceRequest(packet);
            break;

        default:
            stallDefaultControlPipe();
            break;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleClearFeatureDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in ADDRESS or CONFIGURED state.
    // wValue:  Feature selector
    // wIndex:  0
    // wLength: 0
    // Return:  none
    if ((m_state == AddressState || m_state == ConfiguredState)
        && packet.wIndex == 0 && packet.wLength == 0)
    {
        if (packet.wValue == Usb::SetupPacket::DeviceRemoteWakeupFeature)
        {
            m_remoteWakeupEnabled = false;
            //! \todo Inform the class driver
            sendDefaultControlPipeStatus();
            return;
        }
        // Note: It is not possible to clear the Test_Mode feature
        // (USB 2.0 specification, ch 9.4.1).
    }

    stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleGetConfigurationDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in ADDRESS or CONFIGURED state.
    // wValue:  0
    // wIndex:  0
    // wLength: 1
    // Return:  The currently active configuration.
    if (packet.wValue == 0 && packet.wIndex == 0 && packet.wLength == 1)
    {
        //! \todo Should be a member of UsbDevice
        static std::uint32_t configuration = 0;
        if (m_state == AddressState)
        {
            configuration = 0;
            sendDefaultControlPipeData(&configuration, 1);
            return;
        }
        else if (m_state == ConfiguredState)
        {
            configuration = m_activeConfiguration;
            sendDefaultControlPipeData(&configuration, 1);
            return;
        }
    }

    stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleGetDescriptorDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in DEFAULT, ADDRESS or CONFIGURED state.
    // wValue:  descriptor type (high byte) and descriptor index (low byte)
    // wIndex:  0
    // wLength: 1
    // Return:  The currently active configuration.
    enum DescriptorType
    {
        DeviceDescriptor                  = 1,
        ConfigurationDescriptor           = 2,
        StringDescriptor                  = 3,
        InterfaceDescriptor               = 4,
        EndpointDescriptor                = 5,
        DeviceQualifierDescriptor         = 6,
        OtherSpeedConfigurationDescriptor = 7,
        InterfacePowerDescriptor          = 8
    };

    std::uint8_t descriptorType = packet.wValue >> 8;
    std::uint8_t descriptorIndex = packet.wValue & 0x00FF;

    std::pair<const void*, std::uint32_t> descriptor
            = std::pair<const void*, std::uint32_t>(NULL, 0);
    switch (descriptorType)
    {
        case DeviceDescriptor:
            if (descriptorIndex == 0)
            {
                descriptor = m_descriptorProvider->deviceDescriptor(
                                 this->enumeratedSpeed());
            }
            break;

        case ConfigurationDescriptor:
            descriptor = m_descriptorProvider->configurationDescriptor(
                             this->enumeratedSpeed(), descriptorIndex);
            break;

        case StringDescriptor:
            descriptor = m_descriptorProvider->stringDescriptor(
                             this->enumeratedSpeed(), descriptorIndex);
            break;

        case DeviceQualifierDescriptor:
            if (descriptorIndex == 0)
            {
                descriptor = m_descriptorProvider->deviceQualifierDescriptor(
                                 this->enumeratedSpeed());
            }
            break;

        case OtherSpeedConfigurationDescriptor:
            descriptor = m_descriptorProvider->otherSpeedConfigurationDescriptor(
                             this->enumeratedSpeed(), descriptorIndex);
            break;

        default:
            break;
    }

    if (descriptor.first)
    {
        // Do not send more data than the host requested.
        if (descriptor.second > packet.wLength)
            descriptor.second = packet.wLength;
        sendDefaultControlPipeData(descriptor.first, descriptor.second);
    }
    else
        stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleGetStatusDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in ADDRESS or CONFIGURED state.
    // wValue:  0
    // wIndex:  0
    // wLength: 2
    // Return:  16-bit
    //          bit 15-2: 0
    //          bit    1: 1 if remote wakeup is enabled
    //          bit    0: 1 if the device is bus powered
    if ((m_state == AddressState || m_state == ConfiguredState)
        && packet.wValue == 0 && packet.wIndex == 0 && packet.wLength == 2)
    {
        //! \todo Should be a member of UsbDevice with suitable alignment
        //! for DMA (e.g. 4 byte aligned).
        static std::uint32_t status;
        status = 0;
        if (self_type::options::self_powered)
        {
            status |= 0x01;
        }
        if (m_remoteWakeupEnabled)
        {
            status |= 0x02;
        }

        sendDefaultControlPipeData(&status, 2);
    }
    else
    {
        stallDefaultControlPipe();
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleSetAddressDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in DEFAULT or ADDRESS state.
    // wValue:  address (0 .. 127)
    // wIndex:  0
    // wLength: 0
    // Return:  none
    if ((m_state == DefaultState || m_state == AddressState)
        && packet.wValue <= 127 && packet.wIndex == 0 && packet.wLength == 0)
    {
        std::uint8_t address = packet.wValue;

        // If the address is zero, we go back to the DEFAULT state otherwise we
        // are in the ADDRESS state from now on.
        m_state = (address == 0) ? DefaultState : AddressState;

        //! \todo Check the order of the following two function calls with the
        //! USB 2.0 specification 9.4.6.
        this->setDeviceAddress(address);
        sendDefaultControlPipeStatus();
        return;
    }

    stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleSetConfigurationDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in ADDRESS or CONFIGURED state.
    // wValue:  index of new configuration
    // wIndex:  0
    // wLength: 0
    // Return:  none

    // Note: The upper byte of wValue must be zero according to the
    // USB 2.0 specification, ch. 9.4.7.
    if (packet.wValue <= 0xFF && packet.wIndex == 0 && packet.wLength == 0)
    {
        if (m_state == AddressState)
        {
            if (packet.wValue != 0)
            {
                // Let the class driver perform the necessary initialization. It
                // returns true upon success.
                if (derived().classSetConfigurationHandler(packet.wValue))
                {
                    // Set the new configuration and change to the CONFIGURED
                    // state.
                    m_activeConfiguration = packet.wValue;
                    m_state = ConfiguredState;
                    if (m_listener)
                        m_listener->onConfigure();
                    sendDefaultControlPipeStatus();
                }
                else
                    stallDefaultControlPipe();
            }
            else
            {
                // The device must remain in the ADDRESS state.
                sendDefaultControlPipeStatus();
            }
            return;
        }
        else if (m_state == ConfiguredState)
        {
            // Inform the class driver about the de-configuration.
            derived().classClearConfigurationHandler(m_activeConfiguration);
            m_activeConfiguration = 0;
            m_state = AddressState;

            // Activate the new configuration.
            if (packet.wValue != 0)
            {
                if (derived().classSetConfigurationHandler(packet.wValue))
                {
                    m_activeConfiguration = packet.wValue;
                    m_state = ConfiguredState;
                    if (m_listener)
                        m_listener->onConfigure();
                    sendDefaultControlPipeStatus();
                }
                else
                    stallDefaultControlPipe();
            }

            sendDefaultControlPipeStatus();
            return;
        }
    }

    stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::handleSetFeatureDeviceRequest(
        const Usb::SetupPacket& packet)
{
    // Valid in ADDRESS or CONFIGURED state.
    // wValue:  Feature selector
    // wIndex:  0
    // wLength: 0
    // Return:  none
    if ((m_state == AddressState || m_state == ConfiguredState)
        && packet.wIndex == 0 && packet.wLength == 0)
    {
        if (packet.wValue == Usb::SetupPacket::DeviceRemoteWakeupFeature)
        {
            m_remoteWakeupEnabled = true;
            //! \todo Inform the class driver
            sendDefaultControlPipeStatus();
            return;
        }
        //! \todo The Test_Mode feature is not implemented, yet.
    }

    stallDefaultControlPipe();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::dispatchEndpointRequests(
        const Usb::SetupPacket& packet)
{
    switch (packet.bRequest)
    {
        case Usb::SetupPacket::GetStatus:
        {
            // Valid in ADDRESS state (only endpoint 0) or CONFIGURED state (all
            // endpoints).
            // wValue:  0
            // wIndex:  Selects the endpoint.
            // wLength: 2
            // Data:    The endpoint status.
            if (packet.wValue == 0 && packet.wLength == 2)
            {
                //! \todo Move somewhere
                static std::uint32_t endpointStatus;
                endpointStatus = this->endpoint(packet.wIndex).stalled();

                if ((m_state == AddressState && (packet.wIndex & 0x7F) == 0)
                    || m_state == ConfiguredState)
                {
                    sendDefaultControlPipeData(&endpointStatus, 2);
                    return;
                }
            }
        } break;

        case Usb::SetupPacket::ClearFeature:
        {
            // Valid in ADDRESS state (only endpoint 0) or CONFIGURED state (all
            // endpoints).
            // wValue:  Feature selector
            // wIndex:  Selects the endpoint.
            // wLength: 0
            // Data:    none
            if (packet.wValue == Usb::SetupPacket::EndpointHaltFeature
                && packet.wLength == 0)
            {
                if ((packet.wIndex & 0x7F) == 0)
                {
                    // Endpoint IN 0 or OUT 0 has been addressed.
                    if (m_state == AddressState || m_state == ConfiguredState)
                    {
                        sendDefaultControlPipeStatus();
                        return;
                    }
                }
                else
                {
                    // The other endpoints are handled by the USB class-driver.
                    derived().classStandardSetupPacketHandler(packet);
                    return;
                }
            }
        } break;

        case Usb::SetupPacket::SetFeature:
        {
            // Valid in ADDRESS state (only endpoint 0) or CONFIGURED state (all
            // endpoints).
            // wValue:  Feature selector
            // wIndex:  Selects the endpoint.
            // wLength: 0
            // Data:    none
            if (packet.wValue == Usb::SetupPacket::EndpointHaltFeature
                && packet.wLength == 0)
            {
                if ((packet.wIndex & 0x7F) == 0)
                {
                    // Endpoint IN 0 or OUT 0 has been addressed.
                    // Note that it is not necessary to stall endpoint IN 0
                    // or OUT 0.
                    //! \todo Quote the USB specification
                    if (m_state == AddressState || m_state == ConfiguredState)
                    {
                        sendDefaultControlPipeStatus();
                        return;
                    }
                }
                else
                {
                    // The other endpoints are handled by the USB class-driver.
                    derived().classStandardSetupPacketHandler(packet);
                    return;
                }
            }
        } break;

        default:
            break;
    }

    stallDefaultControlPipe();
}

// ----=====================================================================----
//     Handlers invoked from the (low-level) core driver
// ----=====================================================================----

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceConnectHandler()
{
    if (m_listener)
        m_listener->onConnect();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceDisconnectHandler()
{
    if (m_listener)
        m_listener->onDisconnect();
    derived().classClearConfigurationHandler(m_activeConfiguration);
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceEnumerationCompletedHandler()
{
    // Reconfigure the endpoints OUT 0 and IN 0 according to the enumerated
    // speed.
    switch (this->enumeratedSpeed())
    {
        default:
        case Usb::LowSpeed:
        {
            // The maximum packet size for low-speed USB is 8 bytes.
            this->openEndpoint(0x00, 8, Usb::ControlEndpoint);
            this->openEndpoint(0x80, 8, Usb::ControlEndpoint);
        } break;

        case Usb::FullSpeed:
        {
            //! \todo Should get the packet size from the descriptor
            // The maximum packet size for full-speed USB is 64 bytes.
            this->openEndpoint(0x00, 64, Usb::ControlEndpoint);
            this->openEndpoint(0x80, 64, Usb::ControlEndpoint);
        } break;

        case Usb::HighSpeed:
        {
            // The maximum packet size for high-speed USB is 64 bytes.
            this->openEndpoint(0x00, 64, Usb::ControlEndpoint);
            this->openEndpoint(0x80, 64, Usb::ControlEndpoint);
        } break;
    }

    // Prepare the endpoint OUT 0 (belonging to the Default Control Pipe) to
    // receive a SETUP packet.
    m_defaultControlPipeState = DcpIdle;
    this->prepareSetupPacketReception(&m_setupPacket, 24);

    if (m_listener)
        m_listener->onEnumerationComplete(this->enumeratedSpeed());
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceIncompleteIsochronousInHandler()
{
    derived().classIncompleteIsochronousInHandler();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceIncompleteIsochronousOutHandler()
{
    derived().classIncompleteIsochronousOutHandler();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceInTransactionCompletedHandler(
        std::uint8_t endpointNumber, std::uint32_t numBytesSent)
{
    if (endpointNumber == 0)
    {
        if (   m_defaultControlPipeState == DcpDataInStage
            || m_defaultControlPipeState == DcpSignallingDataInStage)
        {
            if (numBytesSent > m_dcpInTransfer.numRemainingBytes)
            {
                stallDefaultControlPipe();
                return;
            }

            m_dcpInTransfer.numRemainingBytes -= numBytesSent;
            if (m_dcpInTransfer.numRemainingBytes)
            {
                // Initiate another transfer only if the host has read all the
                // data during the previous one.
                std::uint32_t maxTransactionSize
                        = this->endpoint(0x80).maxTransactionSize();
                if (numBytesSent == maxTransactionSize)
                {
                    this->sendData(0x80, NULL,
                                   std::min(m_dcpInTransfer.numRemainingBytes,
                                            maxTransactionSize));
                }
                else
                {
                    stallDefaultControlPipe();
                }
            }
            else
            {
                // If the total number of bytes which have been transferred is
                // a multiple of the packet size, the transfer has to be
                // terminated with a zero-length packet.
                if (m_dcpInTransfer.bufferSize != 0
                    && m_dcpInTransfer.bufferSize
                       % this->endpoint(0x80).maxPacketSize() == 0)
                {
                    // Next time we must not send a zero-length packet.
                    m_dcpInTransfer.bufferSize = 0;
                    this->sendData(0x80, NULL, 0);
                }
                else
                {
                    // The transfer is over. If it is signalling, the class
                    // driver has to be informed right now.
                    if (m_defaultControlPipeState == DcpDataInStage
                        || derived().classDefaultControlPipeDataSentHandler())
                    {
                        prepareDefaultControlPipeStatusReception();
                    }
                    else
                    {
                        stallDefaultControlPipe();
                    }
                }
            }
        }
        else if (m_defaultControlPipeState == DcpStatusInStage)
        {
            // The control transfer has been completed.
            m_defaultControlPipeState = DcpIdle;
            this->prepareSetupPacketReception(
                        &m_setupPacket, sizeof(m_setupPacket));
        }
        else
        {
            // Ooops... we do not expect an IN data packet right now.
            stallDefaultControlPipe();
        }
    }
    else
    {
        derived().classDataSentHandler(endpointNumber, numBytesSent);
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceOutTransactionCompletedHandler(
        std::uint8_t endpointNumber, std::uint32_t numBytesReceived)
{
    if (endpointNumber == 0)
    {
        if (   m_defaultControlPipeState == DcpDataOutStage
            || m_defaultControlPipeState == DcpSignallingDataOutStage)
        {
            if (numBytesReceived > m_dcpOutTransfer.numRemainingBytes)
            {
                stallDefaultControlPipe();
                return;
            }

            m_dcpOutTransfer.numRemainingBytes -= numBytesReceived;
            if (m_dcpOutTransfer.numRemainingBytes)
            {
                // Do not initiate another transfer, if the host sent a short
                // packet in the previous one.
                std::uint32_t maxTransactionSize
                        = this->endpoint(0x00).maxTransactionSize();
                if (numBytesReceived == maxTransactionSize)
                {
                    this->prepareReception(
                            0x00, NULL,
                            std::min(m_dcpOutTransfer.numRemainingBytes,
                                     maxTransactionSize));
                }
                else
                {
                    stallDefaultControlPipe();
                }
            }
            else
            {
                // We have received the whole data. If this is a signalling
                // transfer, the class driver has to be informed right now.
                if (m_defaultControlPipeState == DcpDataOutStage
                    || derived().classDefaultControlPipeDataReceivedHandler())
                {
                    sendDefaultControlPipeStatus();
                }
                else
                {
                    stallDefaultControlPipe();
                }
            }
        }
        else if (m_defaultControlPipeState == DcpStatusOutStage)
        {
            // The control transfer has been completed.
            m_defaultControlPipeState = DcpIdle;
            this->prepareSetupPacketReception(
                        &m_setupPacket, sizeof(m_setupPacket));
        }
        else
        {
            // Hm... we are neither in the DATA stage nor in the STATUS stage
            // but have received a data OUT packet.
            stallDefaultControlPipe();
        }
    }
    else
    {
        // Notify the USB class driver about the completion of a transfer
        // which happened not over the Default Control Pipe.
        derived().classDataReceivedHandler(endpointNumber, numBytesReceived);
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceResumeHandler()
{
    m_state = m_preSuspendState;
    if (m_listener)
        m_listener->onResume();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceSetupPacketHandler(
        std::uint8_t address, std::uint8_t numSetupPackets)
{
    // We only care about SETUP packets belonging to the Default Control Pipe.
    if (address != 0)
        return;

    // A SETUP packet cancels any data or status stage.
    m_defaultControlPipeState = DcpSetupStage;

    this->clearEndpointStall(0x00);
    this->clearEndpointStall(0x80);

    const Usb::SetupPacket& setupPacket
            = *static_cast<const Usb::SetupPacket*>(
                  static_cast<void*>(&m_setupPacket[numSetupPackets - 1]));

    if ((setupPacket.bmRequestType & Usb::SetupPacket::TypeMask)
        == Usb::SetupPacket::TypeStandard)
    {
        switch (setupPacket.bmRequestType & Usb::SetupPacket::RecipientMask)
        {
            case Usb::SetupPacket::RecipientDevice:
                dispatchStandardDeviceRequests(setupPacket);
                break;

            case Usb::SetupPacket::RecipientInterface:
                // Interface requests are handled by the USB class-driver.
                derived().classStandardSetupPacketHandler(setupPacket);
                break;

            case Usb::SetupPacket::RecipientEndpoint:
                dispatchEndpointRequests(setupPacket);
                break;

            default:
                stallDefaultControlPipe();
                break;
        }
    }
    else
    {
        // The SETUP packet is of class- or vendor-type. In both cases we let
        // the USB class-driver handle the request.
        derived().classNonStandardSetupPacketHandler(setupPacket);
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceStartOfFrameHandler()
{
    derived().classStartOfFrameHandler();
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceSuspendHandler()
{
    if (m_listener)
        m_listener->onSuspend();
    m_preSuspendState = m_state;
    m_state = SuspendedState;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDevice<TUsbSelector, TDerived, TOptions>::deviceUsbResetHandler()
{
    m_state = DefaultState;

    // Open the endpoint OUT 0.
    this->openEndpoint(0x00, 64, Usb::ControlEndpoint);//! \todo Get max size from traits
    // Open the endpoint IN 0.
    this->openEndpoint(0x80, 64, Usb::ControlEndpoint);//! \todo Get max size from traits

    // Prepare the endpoint OUT 0 (belonging to the Default Control Pipe) to
    // receive a SETUP packet.
    m_defaultControlPipeState = DcpIdle;
    this->prepareSetupPacketReception(&m_setupPacket, 24);

    if (m_listener)
        m_listener->onReset();
}

} // namespace m15

#endif // M15_USBDEVICE_HPP
