#ifndef M15_USBCOMMON_HPP
#define M15_USBCOMMON_HPP

#include <cstdint>

namespace m15
{
namespace Usb
{

enum Speed
{
    LowSpeed,
    FullSpeed,
    HighSpeed
};

enum DescriptorType
{
    DeviceDescriptor        = 0x01,
    ConfigurationDescriptor = 0x02,
    StringDescriptor        = 0x03,
    InterfaceDescriptor     = 0x04,
    EndpointDescriptor      = 0x05
};

enum EndpointDirection
{
    EndpointDirectionOut  = 0x00,
    EndpointDirectionIn   = 0x80,
    EndpointDirectionMask = 0x80
};

enum EndpointType
{
    ControlEndpoint     = 0,
    IsochronousEndpoint = 1,
    BulkEndpoint        = 2,
    InterruptEndpoint   = 3
};

struct SetupPacket
{
    enum Direction
    {
        DirectionOut  = 0x00,
        DirectionIn   = 0x80,
        DirectionMask = 0x80
    };

    enum Recipient
    {
        RecipientDevice    = 0x00,
        RecipientInterface = 0x01,
        RecipientEndpoint  = 0x02,
        RecipientOther     = 0x03,
        RecipientMask      = 0x1F
    };

    enum Type
    {
        TypeStandard = 0x00,
        TypeClass    = 0x20,
        TypeVendor   = 0x40,
        TypeMask     = 0x60
    };

    enum StandardDeviceRequest
    {
        GetStatus        =  0,
        ClearFeature     =  1,
        SetFeature       =  3,
        SetAddress       =  5,
        GetDescriptor    =  6,
        SetDescriptor    =  7,
        GetConfiguration =  8,
        SetConfiguration =  9,
        GetInterface     = 10,
        SetInterface     = 11,
        SynchFrame       = 12
    };

    enum FeatureSelector
    {
        EndpointHaltFeature       = 0,
        DeviceRemoteWakeupFeature = 1,
        TestModeFeature           = 2
    };

    std::uint8_t bmRequestType;
    std::uint8_t bRequest;
    std::uint16_t wValue;
    std::uint16_t wIndex;
    std::uint16_t wLength;
};

} // namespace Usb

//! Describes an end-point.
class UsbEndpoint
{
public:
    //! Returns the direction of the endpoint.
    Usb::EndpointDirection direction() const
    {
        return Usb::EndpointDirection(m_address & Usb::EndpointDirectionMask);
    }

    //! Returns the maximum size (in bytes) of a packet which can be sent or
    //! received via this endpoint.
    std::uint16_t maxPacketSize() const
    {
        return m_maxPacketSize;
    }

    //! The maximum size (in bytes) which can be transferred in a single
    //! transaction.
    std::uint32_t maxTransactionSize() const
    {
        return m_maxTransactionSize;
    }

    //! Sets the address to \p address.
    void setAddress(std::uint8_t address)
    {
        m_address = address;
    }

    //! Sets the maximum packet size to \p size (in bytes).
    void setMaxPacketSize(std::uint16_t size)
    {
        m_maxPacketSize = size;
    }

    void setMaxTransactionSize(std::uint32_t size)
    {
        m_maxTransactionSize = size;
    }

    void setStallFlag(bool stalled)
    {
        m_stalled = stalled;
    }

    //! Sets the endpoint type to \p type.
    void setType(Usb::EndpointType type)
    {
        m_type = type;
    }

    bool stalled() const
    {
        return m_stalled;
    }

    //! Returns the type of the endpoint.
    Usb::EndpointType type() const
    {
        return m_type;
    }

private:
    //! The address of the endpoint, which is a combination of the endpoint's
    //! number and its direction.
    std::uint8_t m_address;

    //! Set if the endpoint has been stalled.
    bool m_stalled;

    //! The type of the endpoint.
    Usb::EndpointType m_type;

    //! The maximum size of a packet which can be transferred over this
    //! endpoint.
    std::uint16_t m_maxPacketSize;

    //! The maximum size (in bytes) which can be transferred in a single
    //! transaction.
    //! \note This maximum transaction size is never less than the maximum
    //! packet size. However, it can be higher if the hardware supports sending
    //! more than one packet in a single request.
    std::uint32_t m_maxTransactionSize;
};

} // namespace m15

#endif // M15_USBCOMMON_HPP
