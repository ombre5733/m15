#ifndef M15_STM32F4XX_USBDEVICECOREDRIVER_HPP
#define M15_STM32F4XX_USBDEVICECOREDRIVER_HPP

//! \todo Needs to go into the config header.
#ifndef __packed
#  define __packed   __attribute__((packed))
#endif

#include "../usbcommon.hpp"

#include <thread.hpp>

#include <stm32f4xx.h>

#include <cstdint>
#include <cstring>

// ----=====================================================================----
//     Register definitions
// ----=====================================================================----

#define GOTGINT_SEDET            ((uint32_t)0x00000004)

#define GAHBCFG_GINT             ((uint32_t)0x00000001)
#define GAHBCFG_HBSTLEN          ((uint32_t)0x0000001E)
#define GAHBCFG_HBSTLEN_INCR     ((uint32_t)0x00000002)
#define GAHBCFG_HBSTLEN_INCR4    ((uint32_t)0x00000006)
#define GAHBCFG_HBSTLEN_INCR8    ((uint32_t)0x0000000A)
#define GAHBCFG_HBSTLEN_INCR16   ((uint32_t)0x0000000E)
#define GAHBCFG_DMAEN            ((uint32_t)0x00000020)
#define GAHBCFG_TXFELVL          ((uint32_t)0x00000080)
#define GAHBCFG_PTXFELVL         ((uint32_t)0x00000100)

#define GUSBCFG_PHSEL            ((uint32_t)0x00000040)
#define GUSBCFG_TRDT             ((uint32_t)0x00003C00)
#define GUSBCFG_FHMOD            ((uint32_t)0x20000000)
#define GUSBCFG_FDMOD            ((uint32_t)0x40000000)

#define GRSTCTL_CSRST            ((uint32_t)0x00000001)
#define GRSTCTL_HSRST            ((uint32_t)0x00000002)
#define GRSTCTL_FCRST            ((uint32_t)0x00000004)
#define GRSTCTL_RXFFLSH          ((uint32_t)0x00000010)
#define GRSTCTL_TXFFLSH          ((uint32_t)0x00000020)
#define GRSTCTL_TXFNUM           ((uint32_t)0x000007C0)
#define GRSTCTL_AHBIDL           ((uint32_t)0x80000000)

#define GINTSTS_CMOD             ((uint32_t)0x00000001)
#define GINTSTS_MMIS             ((uint32_t)0x00000002)
#define GINTSTS_OTGINT           ((uint32_t)0x00000004)
#define GINTSTS_SOF              ((uint32_t)0x00000008)
#define GINTSTS_RXFLVL           ((uint32_t)0x00000010)
#define GINTSTS_GINAKEFF         ((uint32_t)0x00000040)
#define GINTSTS_GONAKEFF         ((uint32_t)0x00000080)
#define GINTSTS_USBSUSP          ((uint32_t)0x00000800)
#define GINTSTS_USBRST           ((uint32_t)0x00001000)
#define GINTSTS_ENUMDNE          ((uint32_t)0x00002000)
#define GINTSTS_IEPINT           ((uint32_t)0x00040000)
#define GINTSTS_OEPINT           ((uint32_t)0x00080000)
#define GINTSTS_INCOMPISOIN      ((uint32_t)0x00100000)
#define GINTSTS_INCOMPISOOUT     ((uint32_t)0x00200000)
#define GINTSTS_SRQINT           ((uint32_t)0x40000000)
#define GINTSTS_WKUINT           ((uint32_t)0x80000000)

#define GINTMSK_MMISM            ((uint32_t)0x00000002)
#define GINTMSK_OTGINTM          ((uint32_t)0x00000004)
#define GINTMSK_SOFM             ((uint32_t)0x00000008)
#define GINTMSK_RXFLVLM          ((uint32_t)0x00000010)
#define GINTMSK_NPTXFEM          ((uint32_t)0x00000020)
#define GINTMSK_GINAKEFFM        ((uint32_t)0x00000040)
#define GINTMSK_GONAKEFFM        ((uint32_t)0x00000080)
#define GINTMSK_ESUSPM           ((uint32_t)0x00000400)
#define GINTMSK_USBSUSPM         ((uint32_t)0x00000800)
#define GINTMSK_USBRSTM          ((uint32_t)0x00001000)
#define GINTMSK_ENUMDNEM         ((uint32_t)0x00002000)
#define GINTMSK_ISOODRPM         ((uint32_t)0x00004000)
#define GINTMSK_EOPFM            ((uint32_t)0x00008000)
#define GINTMSK_EPMISM           ((uint32_t)0x00020000)
#define GINTMSK_IEPINTM          ((uint32_t)0x00040000)
#define GINTMSK_OEPINTM          ((uint32_t)0x00080000)
#define GINTMSK_IISOIXFRM        ((uint32_t)0x00100000)
#define GINTMSK_IISOOXFRM        ((uint32_t)0x00200000)
#define GINTMSK_FSUSPM           ((uint32_t)0x00400000)
#define GINTMSK_PRTIM            ((uint32_t)0x01000000)
#define GINTMSK_HCIM             ((uint32_t)0x02000000)
#define GINTMSK_PTXFEM           ((uint32_t)0x04000000)
#define GINTMSK_CIDSCHGM         ((uint32_t)0x10000000)
#define GINTMSK_DISCINTM         ((uint32_t)0x20000000)
#define GINTMSK_SRQIM            ((uint32_t)0x40000000)
#define GINTMSK_WUIM             ((uint32_t)0x80000000)

#define GRXSTSP_EPNUM            ((uint32_t)0x0000000F)
#define GRXSTSP_BCNT             ((uint32_t)0x00007FF0)
#define GRXSTSP_PKTSTS           ((uint32_t)0x001E0000)

#define GCCFG_PWRDWN             ((uint32_t)0x00010000)
#define GCCFG_I2CPADEN           ((uint32_t)0x00020000)
#define GCCFG_VBUSASEN           ((uint32_t)0x00040000)
#define GCCFG_VBUSBSEN           ((uint32_t)0x00080000)
#define GCCFG_SOFOUTEN           ((uint32_t)0x00100000)
#define GCCFG_NOVBUSSENS         ((uint32_t)0x00200000)

#define DCFG_DSPD                ((uint32_t)0x00000003)
#define DCFG_DSPD_HIGH           ((uint32_t)0x00000000)
#define DCFG_DSPD_FULL           ((uint32_t)0x00000003)
#define DCFG_DAD                 ((uint32_t)0x000007F0)
#define DCFG_PFIVL               ((uint32_t)0x00001800)
#define DCFG_PFIVL_80            ((uint32_t)0x00000000)
#define DCFG_PFIVL_85            ((uint32_t)0x00000800)
#define DCFG_PFIVL_90            ((uint32_t)0x00001000)
#define DCFG_PFIVL_95            ((uint32_t)0x00001800)

#define DCTL_RWUSIG              ((uint32_t)0x00000001)
#define DCTL_SDIS                ((uint32_t)0x00000002)
#define DCTL_GINSTS              ((uint32_t)0x00000004)
#define DCTL_GONSTS              ((uint32_t)0x00000008)
#define DCTL_TCTL                ((uint32_t)0x00000070)
#define DCTL_SGINAK              ((uint32_t)0x00000080)
#define DCTL_CGINAK              ((uint32_t)0x00000100)
#define DCTL_SGONAK              ((uint32_t)0x00000200)
#define DCTL_CGONAK              ((uint32_t)0x00000400)
#define DCTL_POPRGDNE            ((uint32_t)0x00000800)

#define DSTS_SUSPSTS             ((uint32_t)0x00000001)
#define DSTS_ENUMSPD             ((uint32_t)0x00000006)
#define DSTS_ENUMSPD_HIGH        ((uint32_t)0x00000000)
#define DSTS_ENUMSPD_FULL        ((uint32_t)0x00000006)

#define DIEPMSK_XFRCM            ((uint32_t)0x00000001)
#define DIEPMSK_EPDM             ((uint32_t)0x00000002)
#define DIEPMSK_TOM              ((uint32_t)0x00000008)

#define DOEPMSK_XFRCM            ((uint32_t)0x00000001)
#define DOEPMSK_EPDM             ((uint32_t)0x00000002)
#define DOEPMSK_STUPM            ((uint32_t)0x00000008)

#define DIEPCTL0_MPSIZ           ((uint32_t)0x00000003)
#define DIEPCTL0_MPSIZ_64        ((uint32_t)0x00000000)
#define DIEPCTL0_MPSIZ_32        ((uint32_t)0x00000001)
#define DIEPCTL0_MPSIZ_16        ((uint32_t)0x00000002)
#define DIEPCTL0_MPSIZ_8         ((uint32_t)0x00000003)

#define DIEPCTLx_MPSIZ           ((uint32_t)0x000007FF)
#define DIEPCTLx_USBAEP          ((uint32_t)0x00008000)
#define DIEPCTLx_EONUM           ((uint32_t)0x00010000)
#define DIEPCTLx_DPID            ((uint32_t)0x00010000)
#define DIEPCTLx_NAKSTS          ((uint32_t)0x00020000)
#define DIEPCTLx_EPTYP           ((uint32_t)0x000C0000)
#define DIEPCTLx_STALL           ((uint32_t)0x00200000)
#define DIEPCTLx_TXFNUM          ((uint32_t)0x03C00000)
#define DIEPCTLx_CNAK            ((uint32_t)0x04000000)
#define DIEPCTLx_SNAK            ((uint32_t)0x08000000)
#define DIEPCTLx_SD0PID          ((uint32_t)0x10000000)
#define DIEPCTLx_SEVNFRM         ((uint32_t)0x10000000)
#define DIEPCTLx_SODDFRM         ((uint32_t)0x20000000)
#define DIEPCTLx_EPDIS           ((uint32_t)0x40000000)
#define DIEPCTLx_EPENA           ((uint32_t)0x80000000)

#define DOEPCTLx_MPSIZ           ((uint32_t)0x000007FF)
#define DOEPCTLx_USBAEP          ((uint32_t)0x00008000)
#define DOEPCTLx_EONUM           ((uint32_t)0x00010000)
#define DOEPCTLx_DPID            ((uint32_t)0x00010000)
#define DOEPCTLx_NAKSTS          ((uint32_t)0x00020000)
#define DOEPCTLx_EPTYP           ((uint32_t)0x000C0000)
#define DOEPCTLx_SNPM            ((uint32_t)0x00100000)
#define DOEPCTLx_STALL           ((uint32_t)0x00200000)
#define DOEPCTLx_CNAK            ((uint32_t)0x04000000)
#define DOEPCTLx_SNAK            ((uint32_t)0x08000000)
#define DOEPCTLx_SD0PID          ((uint32_t)0x10000000)
#define DOEPCTLx_SEVNFRM         ((uint32_t)0x10000000)
#define DOEPCTLx_SD1PID          ((uint32_t)0x20000000)
#define DOEPCTLx_SODDFRM         ((uint32_t)0x20000000)
#define DOEPCTLx_EPDIS           ((uint32_t)0x40000000)
#define DOEPCTLx_EPENA           ((uint32_t)0x80000000)

#define DIEPINTx_XFRC            ((uint32_t)0x00000001)
#define DIEPINTx_EPDISD          ((uint32_t)0x00000002)
#define DIEPINTx_TOC             ((uint32_t)0x00000008)
#define DIEPINTx_INEPNE          ((uint32_t)0x00000040)
#define DIEPINTx_TXFE            ((uint32_t)0x00000080)

#define DOEPINTx_XFRC            ((uint32_t)0x00000001)
#define DOEPINTx_EPDISD          ((uint32_t)0x00000002)
#define DOEPINTx_STUP            ((uint32_t)0x00000008)

#define DIEPTSIZx_XFRSIZ         ((uint32_t)0x0007FFFF)
#define DIEPTSIZx_MCNT           ((uint32_t)0x60000000)
#define DIEPTSIZx_MCNT_1         ((uint32_t)0x20000000)
#define DIEPTSIZx_MCNT_2         ((uint32_t)0x40000000)
#define DIEPTSIZx_MCNT_3         ((uint32_t)0x60000000)

#define DTXFSTSx_INEPTFSAV       ((uint32_t)0x0000FFFF)

#define DOEPTSIZx_XFRSIZ         ((uint32_t)0x0007FFFF)
#define DOEPTSIZx_STUPCNT        ((uint32_t)0x60000000)

#define PCGCCTL_STPPCLK          ((uint32_t)0x00000001)
#define PCGCCTL_GATEHCLK         ((uint32_t)0x00000002)
#define PCGCCTL_PHYSUSP          ((uint32_t)0x00000010)

namespace m15
{
//! A type tag to select the full-speed USB core.
struct usb_full_speed_t {};
//! A type tag to select the high-speed USB core.
struct usb_high_speed_t {};

namespace detail
{

//! The base class for all low-level hardware dependent USB drivers.
class UsbCoreDriver
{
    public:
        virtual ~UsbCoreDriver() {}
        virtual void handleInterrupt() = 0;
};

//! Traits for the USB core.
template <typename TUsbSelector>
struct usb_core_traits;

// Traits for the full-speed USB core.
template <>
struct usb_core_traits<usb_full_speed_t>
{
    typedef OTG_FS_TypeDef regs_type;

    //! The interrupt number of the USB-FS-OTG core.
    static const unsigned irq_number = OTG_FS_IRQn;

    //! The DMA is only available for the high-speed peripheral.
    static const bool has_dma = false;

    //! The minimum alignment which is required for the data buffers.
    static const unsigned min_data_alignment = 1;

    //! The maximum number of IN endpoints supported by the USB core.
    static const unsigned max_num_in_endpoints = 4;

    //! The maximum number of OUT endpoints supported by the USB core.
    static const unsigned max_num_out_endpoints = 4;

    //! The total size of the FIFO in bytes.
    static const unsigned total_fifo_size = 1280;

    //! Returns the register set of the USB core.
    static regs_type* regs()
    {
        return OTG_FS;
    }

    //! Returns the DIEPCTL register for the \p number-th endpoint.
    static volatile std::uint32_t& DIEPCTLx(std::uint8_t number)
    {
        // The DIEPCTLx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DIEPCTL0 + 8 * number);
    }

    //! Returns the DOEPCTL register for the \p number-th endpoint.
    static volatile std::uint32_t& DOEPCTLx(std::uint8_t number)
    {
        // The DOEPCTLx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DOEPCTL0 + 8 * number);
    }

    //! Returns the DIEPINT register for the \p number-th endpoint.
    static volatile std::uint32_t& DIEPINTx(std::uint8_t number)
    {
        // The DIEPINTx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DIEPINT0 + 8 * number);
    }

    //! Returns the DOEPINT register for the \p number-th endpoint.
    static volatile std::uint32_t& DOEPINTx(std::uint8_t number)
    {
        // The DOEPINTx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DOEPINT0 + 8 * number);
    }

    //! Returns the DIEPTSIZ register for the \p number-th endpoint.
    static volatile std::uint32_t& DIEPTSIZx(std::uint8_t number)
    {
        // The DIEPTSIZx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DIEPTSIZ0 + 8 * number);
    }

    //! Returns the DTXFSTS register for the \p number-th endpoint.
    static volatile std::uint32_t& DTXFSTSx(std::uint8_t number)
    {
        // The DTXFSTSx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DTXFSTS0 + 8 * number);
    }

    //! Returns the DOEPTSIZ register for the \p number-th endpoint.
    static volatile std::uint32_t& DOEPTSIZx(std::uint8_t number)
    {
        // The DOEPTSIZx registers have a spacing of 0x20 bytes (8 words).
        return *(&regs()->DOEPTSIZ0 + 8 * number);
    }

    //! Sets the DIEPDMA register for the \p number-th endpoint to \p addr.
    static void setDIEPDMAx(std::uint8_t number, std::uint32_t addr)
    {
        // The DIEPDMAx registers have a spacing of 0x20 bytes (8 words).
        //*(volatile std::uint32_t*)(&regs()->DIEPDMA0 + 8 * number) = addr;
    }

    //! Sets the DOEPDMA register for the \p number-th endpoint to \p addr.
    static void setDOEPDMAx(std::uint8_t number, std::uint32_t addr)
    {
        // The DOEPDMAx registers have a spacing of 0x20 bytes (8 words).
        //*(volatile std::uint32_t*)(&regs()->DOEPDMA0 + 8 * number) = addr;
    }

    //! Returns a reference to the receive FIFO.
    static volatile std::uint32_t& RXFIFO()
    {
        return *OTG_FS_DFIFO0;
    }

    //! Returns a reference to the transmit FIFO for the \p number-th endpoint.
    static volatile std::uint32_t& TXFIFOx(std::uint8_t number)
    {
        return *(volatile std::uint32_t*)(
                    OTG_FS_BASE + 0x1000 + number * 0x1000);
    }

    static UsbCoreDriver* coreDriver;
};

//! The default options for the low-level USB device core driver. The options
//! are templated on the USB core.
template <typename TUsbSelector>
struct default_usb_device_core_driver_options;

// Specialization for the full-speed USB core.
template <>
struct default_usb_device_core_driver_options<usb_full_speed_t>
{
    //! If set, the PHY clock and the USB core clock are disabled when a USB
    //! suspend event is detected.
    static const bool disable_clock_when_suspended = false;

    //! If set, the USB core monitors the Vbus voltage. \todo: Describe more
    static const bool enable_vbus_monitoring = true;

    //! The size (in bytes) of the receive FIFO, which is shared by all
    //! endpoints.
    static const std::uint32_t rx_fifo_size = 256;

    //! The size (in bytes) of the transmit FIFO for endpoint IN 0.
    static const std::uint32_t tx0_fifo_size = 256;

    //! The size (in bytes) of the transmit FIFO for endpoint IN 1.
    static const std::uint32_t tx1_fifo_size = 256;

    //! The size (in bytes) of the transmit FIFO for endpoint IN 2.
    static const std::uint32_t tx2_fifo_size = 256;

    //! The size (in bytes) of the transmit FIFO for endpoint IN 3.
    static const std::uint32_t tx3_fifo_size = 256;
};

// Specialization for the high-speed USB core.
template <>
struct default_usb_device_core_driver_options<usb_high_speed_t>
{
    enum Physical
    {
        EmbeddedPhysical,
        UlpiPhysical
    };

    //! If set, the DMA data transfer is enabled.
    static const bool enable_dma = false;

    //! If set, the PHY clock and the USB core clock are disabled when a USB
    //! suspend event is detected.
    static const bool disable_clock_when_suspended = false;

    //! If set, the USB core monitors the Vbus voltage. \todo: Describe more
    static const bool enable_vbus_monitoring = true;

    //! \todo Only relevant for high-speed.
    static const Physical physical = EmbeddedPhysical;

    //! \todo Only relevant for high-speed with ULPI.
    static const bool enable_internal_vbus = false;
};

//! Hardware dependent USB device driver.
//!
//! Callbacks are implemented using static polymorphism via the curiously
//! recurring template pattern (CRTP). The USB device driver has to derive
//! from this core driver. It has to pass its type in the \p TDerived
//! parameter. The derived class must implement the following methods:
//! - void deviceStartOfFrameHandler()
//! - void deviceConnectHandler()
//! - void deviceIncompleteIsochronousInHandler()
//! - void deviceIncompleteIsochronousOutHandler()
//! - void deviceDisconnectHandler()
//! - void deviceResumeHandler()
//! - void deviceSuspendHandler()
//!
//!
//! - <tt>void deviceSetupPacketHandler(std::uint8_t address, std::uint8_t numSetupPackets)</tt>
//!   Used to signal that one or more SETUP packets has been received over the
//!   endpoint with the given \p address. The exact number of SETUP packets
//!   is passed in the parameter \p numSetupPackets.
//!   Note: According to the USB 2.0 specification, up to three SETUP packets
//!   can be received back-to-back.
//! - void deviceUsbResetHandler()
//!   Called when a reset event is detected on the USB.
//! - void deviceInTransactionCompletedHandler(
//!       std::uint8_t endpointNumber, std::uint32_t numBytesSent)
//! - void deviceOutTransactionCompletedHandler(
//!       std::uint8_t endpointNumber, std::uint32_t numBytesReceived)
template <typename TUsbSelector,
          typename TDerived,
          typename TOptions = default_usb_device_core_driver_options<TUsbSelector> >
class UsbDeviceCoreDriver : public UsbCoreDriver
{
public:
    //! The traits for this device core driver.
    typedef usb_core_traits<TUsbSelector> traits;
    //! The options which have been passed to the device core driver.
    typedef TOptions options;

    UsbDeviceCoreDriver()
    {
        traits::coreDriver = this;

        initEndpoints();
        initCore();
    }

    virtual ~UsbDeviceCoreDriver()
    {
        disable();
        traits::coreDriver = 0;
    }

    //! Clears the STALL flag of the endpoint with the given \p address.
    void clearEndpointStall(std::uint8_t address);

    //! Closes the endpoint with the given \p address.
    void closeEndpoint(std::uint8_t address);

    void disable();

    void enable();

    //! Returns the settings of the endpoint with the given \p address.
    const UsbEndpoint& endpoint(std::uint8_t address) const;

    //! Opens and resets the endpoint specified with its \p address. The
    //! maximum size of a packet which can be transmitted over the endpoint
    //! is set to \p maxPacketSize (in bytes). The \p type gives the type of
    //! the endpoint.
    void openEndpoint(std::uint8_t address,
                      std::uint16_t maxPacketSize,
                      Usb::EndpointType type);

    //! Sets the address of the USB device to \p address, which has to be in
    //! the range from 0 to 127.
    void setDeviceAddress(std::uint8_t address);

    //! Sets the STALL flag of the endpoint with the given \p address.
    void setEndpointStall(std::uint8_t address);

    //! Returns the enumerated speed.
    //! Returns the speed which has been chosen during device enumeration.
    inline
    Usb::Speed enumeratedSpeed() const
    {
        return m_enumeratedSpeed;
    }

    void signalRemoteWakeup();


    //! Starts to transmit \p bufferSize bytes from the given \p data over the
    //! IN endpoint specified by its \p address. Note that if \p data is a
    //! null-pointer, the transmission will continue from that position where
    //! the previous transmission ended.
    //!
    //! \note The size of the data to transmit must not exceed the maximum
    //! size of a transaction. To send larger amounts of data, this method
    //! has to be called multiple times with \p data being a null-pointer
    //! in all calls except the first.
    void sendData(std::uint8_t address, const void* data,
                  std::uint32_t bufferSize);

    //! Prepares the OUT endpoint with the given \p address to receive
    //! up to \p bufferSize bytes. The incoming data is stored in the \p buffer.
    //! Note that if \p buffer is a null-pointer, the new data will be appended
    //! to the data which has been received previously.
    void prepareReception(std::uint8_t address, void* buffer,
                          std::uint32_t bufferSize);

    //! Prepares the endpoint OUT 0 to receive a SETUP packet, i.e. to handle
    //! the request stage of a control transfer.
    void prepareSetupPacketReception(void* buffer, std::uint32_t bufferSize);

private:
    //! Casts this instance to the derived type.
    TDerived& derived()
    {
        return *static_cast<TDerived*>(this);
    }

    //! Casts this instance to the derived type.
    const TDerived& derived() const
    {
        return *static_cast<const TDerived*>(this);
    }

    void initEndpoints();

    //! Handles and dispatches a USB interrupt.
    virtual void handleInterrupt();
    //! Deals with a speed enumeration completed interrupt.
    void handleEnumerationDoneInterrupt();
    //! Deals with an incomplete isochronous IN transfer interrupt.
    void handleIncompleteIsochronousInTransfer();
    //! Deals with an incomplete isochronous OUT transfer interrupt.
    void handleIncompleteIsochronousOutTransfer();
    //! Deals with interrupts associated with an IN endpoint.
    void handleInEndpointInterrupt();
    //! Deals with interrupts associated with an OUT endpoint.
    void handleOutEndpointInterrupt();
    //! Deals with an OTG protocol event interrupt.
    void handleOtgProtocolEvent();
    //! Deals with a USB resume interrupt.
    void handleResume();
    //! Deals with a RX-FIFO non-empty interrupt.
    void handleRxFifoNonEmptyInterrupt();
    //! Deals with a session request interrupt.
    void handleSessionRequest();
    //! Deals with a start of frame interrupt.
    void handleStartOfFrame();
    //! Deals with a USB suspend interrupt.
    void handleSuspend();
    //! Deals with a USB reset interrupt.
    void handleUsbResetInterrupt();

    void initCore();
    //! Fills the transmit FIFO of the IN endpoint with the number
    //! \p inEndpointNumber.
    void fillTransmitFifo(std::uint8_t inEndpointNumber);
    //! Flushes the transmit FIFO of the IN endpoint with the number
    //! \p inEndpointNumber.
    void flushTransmitFifo(std::uint8_t inEndpointNumber);
    //! Reads \p fifoSize bytes from the receive FIFO and stores them in
    //! the given \p buffer whose size is \p bufferSize. If the \p fifoSize
    //! is greater than the \p bufferSize, the remaining FIFO entries are
    //! popped and discarded.
    void readReceiveFifo(std::uint32_t fifoSize, void* buffer,
                         std::uint32_t bufferSize);

    //! A type for endpoint related data.
    struct EndpointData : public UsbEndpoint
    {
        std::uint8_t* buffer;
        std::uint32_t numBytesToTransceive;
        std::uint32_t numRemainingBytes;
    };

    EndpointData m_inEndpoints[traits::max_num_in_endpoints];
    EndpointData m_outEndpoints[traits::max_num_out_endpoints];

    //! The speed which has been selected during USB enumeration.
    Usb::Speed m_enumeratedSpeed;
};

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::clearEndpointStall(
        std::uint8_t address)
{
    if (address & 0x80)
    {
        address &= 0x7F;
        // Clear the STALL flag of an IN endpoint.
        traits::DIEPCTLx(address) &= ~DIEPCTLx_STALL;
        m_inEndpoints[address].setStallFlag(false);
    }
    else
    {
        // Clear the STALL flag of an OUT endpoint. We also reset the DATA-PID
        // flag to zero for bulk and interrupt endpoints.
        EndpointData& endpoint = m_outEndpoints[address];
        if (endpoint.type() >= Usb::BulkEndpoint)
        {
            traits::DOEPCTLx(address) |= DOEPCTLx_SD0PID;
        }
        traits::DOEPCTLx(address) &= ~DOEPCTLx_STALL;
        endpoint.setStallFlag(false);
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::closeEndpoint(
        std::uint8_t address)
{
    if (address & 0x80)
    {
        address &= 0x7F;

        // Disable an IN endpoint (RM0090, rev 5, p. 1342):
        // - Set the SNAK bit in DIEPCTLx.
        // - Wait until the NAK is effective (INEPNE flag in DIEPINTx).
        // - Set the EPDIS and SNAK bits in DIEPCTLx.
        // - Flush the transmit FIFO.
        traits::DIEPCTLx(address) |= DIEPCTLx_SNAK;
        for (std::uint16_t tries = 1024; tries != 0; --tries)
        {
            if (traits::DIEPINTx(address) & DIEPINTx_INEPNE)
                break;
        }
        if (traits::DIEPCTLx(address) & DIEPCTLx_EPENA)
        {
            traits::DIEPCTLx(address) |= DIEPCTLx_EPDIS;
        }
        traits::DIEPCTLx(address) |= DIEPCTLx_SNAK;
        flushTransmitFifo(address);
        traits::DIEPCTLx(address) &= ~DIEPCTLx_USBAEP;
        // We do not want to receive any further interrupts belonging to
        // this endpoint.
        traits::regs()->DAINTMSK &= ~(std::uint32_t(1) << address);
        //! \todo HS needs to disable another interrupt for EP1.

        EndpointData& endpoint = m_inEndpoints[address];
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
    else
    {
        // Disable an OUT endpoint (RM0090, rev 5, p. 1335):
        // - Set the global OUT NAK bit (SGONAK) in DCTL.
        // - Wait until the global OUT NAK is effective (GONAKEFF in GINTSTS).
        // - Disable the endpoint by setting EPDIS and SNAK in DOEPCTLx. Note
        //   that endpoint OUT 0 cannot be disabled.
        // - Wait for EPDISD in DOEPINTx.
        // - Clear the global OUT NAK bit (CGONAK) in DCTL.
        traits::regs()->DCTL |= DCTL_SGONAK;
        for (std::uint16_t tries = 1024; tries != 0; --tries)
        {
            if (traits::regs()->GINTSTS & GINTSTS_GONAKEFF)
                break;
        }
        if (traits::DOEPCTLx(address) & DOEPCTLx_EPENA)
        {
            traits::DOEPCTLx(address) |= DOEPCTLx_EPDIS;
        }
        traits::DOEPCTLx(address) |= DOEPCTLx_SNAK;
        for (std::uint16_t tries = 1024; tries != 0; --tries)
        {
            if (traits::DOEPINTx(address) & DOEPINTx_EPDISD)
                break;
        }
        traits::DOEPCTLx(address) &= ~DOEPCTLx_USBAEP;
        // We do not want to receive any further interrupts belonging to
        // this endpoint.
        traits::regs()->DAINTMSK &= ~(std::uint32_t(1) << (address + 16));
        //! \todo HS needs to disable another interrupt for EP1.
        traits::regs()->DCTL |= DCTL_CGONAK;

        EndpointData& endpoint = m_outEndpoints[address];
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::disable()
{
    //! \todo Make use of the interrupt-lib here.
    NVIC->ICER[traits::irq_number >> 0x05]
            = std::uint32_t(0x01) << (traits::irq_number & 0x1F);
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::enable()
{
    //! \todo Make use of the interrupt-lib here.
    NVIC->ISER[traits::irq_number >> 0x05]
            = std::uint32_t(0x01) << (traits::irq_number & 0x1F);
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
const UsbEndpoint& UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::endpoint(
        std::uint8_t address) const
{
    return (address & 0x80) ? m_inEndpoints[address & 0x7F]
                            : m_outEndpoints[address];
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::openEndpoint(
        std::uint8_t address, std::uint16_t maxPacketSize,
        Usb::EndpointType type)
{
    std::uint8_t endpointNumber = address & 0x7F;

    std::uint32_t dxepctl;
    if (address & 0x80)
        dxepctl = traits::DIEPCTLx(endpointNumber);
    else
        dxepctl = traits::DOEPCTLx(endpointNumber);

    dxepctl &= ~(DIEPCTLx_MPSIZ | DIEPCTLx_EPTYP | DIEPCTLx_TXFNUM
                 | DIEPCTLx_SD0PID | DIEPCTLx_USBAEP);

    if (endpointNumber == 0)
    {
        // There is a special encoding of the maximum packet size for the
        // endpoints OUT 0 and IN 0.
        if (maxPacketSize >= 64)
            dxepctl |= DIEPCTL0_MPSIZ_64;
        else if (maxPacketSize >= 32)
            dxepctl |= DIEPCTL0_MPSIZ_32;
        else if (maxPacketSize >= 16)
            dxepctl |= DIEPCTL0_MPSIZ_16;
        else
            dxepctl |= DIEPCTL0_MPSIZ_8;
    }
    else
    {
        dxepctl |= maxPacketSize;
    }
    dxepctl |= static_cast<std::uint32_t>(type) << 18;
    // Only IN endpoints have a transfer FIFO. However, the corresponding bits
    // for the OUT endpoints are reserved so trying to set them does not hurt.
    dxepctl |= static_cast<std::uint32_t>(endpointNumber) << 22;
    // In a bulk transfer, the first packet is sent as DATA0 and then the
    // DATA bit is toggled.
    dxepctl |= DIEPCTLx_SD0PID;
    dxepctl |= DIEPCTLx_USBAEP;

    // Write the endpoint configuration and unmask the endpoint interrupt.
    if (address & 0x80)
    {
        traits::DIEPCTLx(endpointNumber) = dxepctl;
        traits::regs()->DAINTMSK |= std::uint32_t(1) << endpointNumber;
        //! \todo HS needs to set another interrupt for EP1

        // The endpoints IN 0 and OUT 0 can only transfer a single packet while
        // the others can transfer up to 1023 packets.
        EndpointData& endpoint = m_inEndpoints[endpointNumber];
        endpoint.setMaxPacketSize(maxPacketSize);
        endpoint.setMaxTransactionSize(
                    endpointNumber == 0 ? maxPacketSize
                                        : std::uint32_t(1023) * maxPacketSize);
        endpoint.setType(type);

        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
    else
    {
        traits::DOEPCTLx(endpointNumber) = dxepctl;
        traits::regs()->DAINTMSK |= std::uint32_t(1) << (endpointNumber + 16);
        //! \todo HS needs to set another interrupt for EP1

        // The endpoints IN 0 and OUT 0 can only transfer a single packet while
        // the others can transfer up to 1023 packets.
        EndpointData& endpoint = m_outEndpoints[endpointNumber];
        endpoint.setMaxPacketSize(maxPacketSize);
        endpoint.setMaxTransactionSize(
                    endpointNumber == 0 ? maxPacketSize
                                        : std::uint32_t(1023) * maxPacketSize);
        endpoint.setType(type);

        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::setDeviceAddress(
        std::uint8_t address)
{
    std::uint32_t dcfg = traits::regs()->DCFG;
    dcfg &= ~DCFG_DAD;
    dcfg |= std::uint32_t(address) << 4;
    traits::regs()->DCFG = dcfg;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::setEndpointStall(
        std::uint8_t address)
{
    if (address & 0x80)
    {
        address &= 0x7F;

        // Stalling an IN endpoint (RM0090, rev 5, p. 1347):
        // - If the endpoint is enabled, set the EPDIS bit in DIEPCTLx.
        // - Set the STALL bit in DIEPCTLx.
        // - Flush the transmit FIFO.
        if (traits::DIEPCTLx(address) & DIEPCTLx_EPENA)
        {
            traits::DIEPCTLx(address) |= DIEPCTLx_EPDIS;
        }
        traits::DIEPCTLx(address) |= DIEPCTLx_STALL;
        flushTransmitFifo(address);

        EndpointData& endpoint = m_inEndpoints[address];
        endpoint.setStallFlag(true);
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
    else
    {
        // Stalling an OUT endpoint (RM0090, rev 5, p. 1339):
        // - Set the global OUT NAK bit (SGONAK) in DCTL.
        // - Wait until the global OUT NAK is effective (GONAKEFF in GINTSTS).
        // - Disable the endpoint by setting EPDIS and STALL in DOEPCTLx. Note
        //   that endpoint OUT 0 cannot be disabled.
        // - Wait for EPDISD in DOEPINTx.
        // - Clear the global OUT NAK bit (CGONAK) in DCTL.
        traits::regs()->DCTL |= DCTL_SGONAK;
        for (std::uint16_t tries = 1024; tries != 0; --tries)
        {
            if (traits::regs()->GINTSTS & GINTSTS_GONAKEFF)
                break;
        }
        if (traits::DOEPCTLx(address) & DOEPCTLx_EPENA)
        {
            traits::DOEPCTLx(address) |= DOEPCTLx_EPDIS;
        }
        traits::DOEPCTLx(address) |= DOEPCTLx_STALL;
        if (address)
        {
            for (std::uint16_t tries = 1024; tries != 0; --tries)
            {
                if (traits::DOEPINTx(address) & DOEPINTx_EPDISD)
                    break;
            }
        }
        traits::regs()->DCTL |= DCTL_CGONAK;

        EndpointData& endpoint = m_outEndpoints[address];
        endpoint.setStallFlag(true);
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::signalRemoteWakeup()
{
    //! \todo Has to be implemented
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::sendData(
        std::uint8_t address, const void* data, std::uint32_t bufferSize)
{
    //! \todo Probably assert that address & 0x80 is set because we can only
    //! send via an IN endpoint.

    address &= 0x7F;
    EndpointData& endpoint = m_inEndpoints[address];

    // If the endpoint is not enabled, we cannot transfer the data.
    if ((traits::DIEPCTLx(address) & DIEPCTLx_USBAEP) == 0)
        return;

    // Limit the size to the maximum transaction length.
    std::uint32_t maxLength = endpoint.maxTransactionSize();
    if (bufferSize > maxLength)
        bufferSize = maxLength;

    // Setup the transaction.
    if (data)
    {
        endpoint.buffer = static_cast<std::uint8_t*>(const_cast<void*>(data));
    }
    endpoint.numBytesToTransceive = bufferSize;
    endpoint.numRemainingBytes = bufferSize;

    // If we have to transfer 0 bytes of data, we have to send one zero-length
    // packet. Otherwise, the number of packets is determined by the amount of
    // data to send and the maximum packet size of this endpoint.
    std::uint32_t numPackets = 1;
    if (bufferSize > 0)
    {
        numPackets = (bufferSize + endpoint.maxPacketSize() - 1)
                     / endpoint.maxPacketSize();
    }

    // Set the number of packets and the total transfer size. Note: The
    // multi-count is only needed for isochronous endpoints but it does not harm
    // to set it for other endpoints, too.
    traits::DIEPTSIZx(address) = (address != 0 ? DIEPTSIZx_MCNT_1 : 0)
                                 | (numPackets << 19)
                                 | bufferSize;

    if (traits::has_dma)
    {
        traits::setDIEPDMAx(address, (std::uint32_t)endpoint.buffer);
    }
    else
    {
        if (endpoint.type() != Usb::IsochronousEndpoint && bufferSize > 0)
        {
            // Unmask the IN endpoint FIFO empty interrupt so that the USB core
            // can interrupt the application when the FIFO is empty. We react
            // on this interrupt and fill the FIFO from it.
            traits::regs()->DIEPEMPMSK |= std::uint32_t(1) << address;
        }
    }

    std::uint32_t diepctl = traits::DIEPCTLx(address);

    if (endpoint.type() == Usb::IsochronousEndpoint)
    {
        // Before setting the even/odd frame flag, wait until there is enough
        // space available in the FIFO.
        while ((traits::DTXFSTSx(address) & DTXFSTSx_INEPTFSAV) * 4 < bufferSize);

        if ((traits::regs()->DSTS & 0x0100) == 0)
        {
            diepctl |= DIEPCTLx_SODDFRM;
        }
        else
        {
            diepctl |= DIEPCTLx_SEVNFRM;
        }
    }

    // Clear the NAK bit and enable the transmission.
    diepctl |= DIEPCTLx_CNAK | DIEPCTLx_EPENA;
    traits::DIEPCTLx(address) = diepctl;

    if (endpoint.type() == Usb::IsochronousEndpoint && bufferSize > 0
        && !traits::has_dma)
    {
        fillTransmitFifo(address);
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::prepareReception(
        std::uint8_t address, void* buffer, std::uint32_t bufferSize)
{
    // Receiving data via an OUT endpoint (RM0090, rev 5, p. 1337):
    // - Program the transfer size and packet count in DOEPTSIZx.
    //   Note: Always prepare the USB core to receive a multiple of the
    //   endpoint's maximum packet size. The transfer completed (XFRC)
    //   interrupt in DOEPINTx is set when
    //   * the both the transfer size and the packet count drop to 0 or
    //   * a packet whose size is less than the maximum packet size is received.
    // - Program the DOEPCTLx register, especially EPENA and CNAK have to be
    //   set.
    // - Pop the data from the FIFO when RXFLVL in GINTSTS is set.
    // - If XFRC in DOEPTSIZx is set, the transfer has been completed.
    // - The amount of data which still could be received is stored in
    //   DOEPTSIZx.

    //! \todo: Probably assert that address & 0x80 is not set.
    //! \todo Check if the endpoint is enabled

    EndpointData& endpoint = m_outEndpoints[address];

    // Limit the size to the maximum transaction length.
    std::uint32_t maxLength = endpoint.maxTransactionSize();
    if (bufferSize > maxLength)
        bufferSize = maxLength;

    // Setup the transaction.
    if (buffer)
    {
        endpoint.buffer = static_cast<std::uint8_t*>(buffer);
    }
    endpoint.numBytesToTransceive = bufferSize;
    endpoint.numRemainingBytes = bufferSize;

    // For a zero-length packet, the number of packets has to be set to one.
    // Otherwise, the number of packets is determined by the amount of
    // data to receive and the maximum packet size of this endpoint.
    std::uint32_t numPackets = 1;
    if (bufferSize > 0)
    {
        numPackets = (bufferSize + endpoint.maxPacketSize() - 1)
                     / endpoint.maxPacketSize();
    }

    std::uint32_t transferSize = numPackets * endpoint.maxPacketSize();
    traits::DOEPTSIZx(address) = (numPackets << 19)
                                 | transferSize;

    if (traits::has_dma)
    {
        traits::setDOEPDMAx(address, (std::uint32_t)endpoint.buffer);
    }

    std::uint32_t doepctl = traits::DOEPCTLx(address);
    if (endpoint.type() == Usb::IsochronousEndpoint)
    {
        //! \todo Probably wait until there is enough space in the FIFO.
        //! \todo Implement this
    }
    // Clear the NAK bit and enable the transmission.
    doepctl |= DOEPCTLx_CNAK | DOEPCTLx_EPENA;
    traits::DOEPCTLx(address) = doepctl;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::prepareSetupPacketReception(
        void* buffer, std::uint32_t bufferSize)
{
    if (bufferSize > 24)
        bufferSize = 24;

    EndpointData& endpoint = m_outEndpoints[0];
    endpoint.buffer = static_cast<std::uint8_t*>(buffer);
    endpoint.numBytesToTransceive = bufferSize;
    endpoint.numRemainingBytes = bufferSize;

    // At maximum, we can receive three SETUP packets back-to-back with a total
    // size of 24 bytes.
    std::uint32_t doeptsiz =   (3 << 29)
                               | (1 << 19)
                               | bufferSize;
    traits::DOEPTSIZx(0) = doeptsiz;

    if (traits::has_dma)
    {
        traits::setDOEPDMAx(0, (std::uint32_t)endpoint.buffer);
        traits::DOEPCTLx(0) |= DOEPCTLx_EPENA;
    }
}

// ----=====================================================================----
//     Interrupt handlers
// ----=====================================================================----

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleInterrupt()
{
    // If we are not in device mode, something has gone wrong.
    // We leave the ISR immediately.
    if (traits::regs()->GINTSTS & GINTSTS_CMOD)
        return;

    // Read the interrupt flags in order to handle them.
    std::uint32_t gintsts = traits::regs()->GINTSTS & traits::regs()->GINTMSK;
    if (!gintsts)
        return;


    if (gintsts & GINTSTS_MMIS)
    {
        // Clear the interrupt flag.
        traits::regs()->GINTSTS = GINTSTS_MMIS;
    }

    if (gintsts & GINTSTS_WKUINT)
    {
        handleResume();
    }

    if (gintsts & GINTSTS_USBSUSP)
    {
        handleSuspend();
    }
    if (gintsts & GINTSTS_SOF)
    {
        handleStartOfFrame();
    }

    if (gintsts & GINTSTS_RXFLVL)
    {
        handleRxFifoNonEmptyInterrupt();
    }

    if (gintsts & GINTSTS_USBRST)
    {
        handleUsbResetInterrupt();
    }
    if (gintsts & GINTSTS_ENUMDNE)
    {
        handleEnumerationDoneInterrupt();
    }
    if (gintsts & GINTSTS_IEPINT)
    {
        handleInEndpointInterrupt();
    }
    if (gintsts & GINTSTS_OEPINT)
    {
        handleOutEndpointInterrupt();
    }

    if (gintsts & GINTSTS_INCOMPISOIN)
    {
        handleIncompleteIsochronousInTransfer();
    }

    if (gintsts & GINTSTS_INCOMPISOOUT)
    {
        handleIncompleteIsochronousOutTransfer();
    }

    if (options::enable_vbus_monitoring)
    {
        if (gintsts & GINTSTS_SRQINT)
        {
            handleSessionRequest();
        }
        if (gintsts & GINTSTS_OTGINT)
        {
            handleOtgProtocolEvent();
        }
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleEnumerationDoneInterrupt()
{
    // RM0090, rev 5, p. 1329:
    // - Read the enumeration speed from DSTS.
    // - Program MPSIZ in DIEPCTL0 depending on the speed.
    //   Note: The MPSIZ in DOEPCTL0 is read-only and the same value is used as
    //   for DIEPCTL0.

    switch (traits::regs()->DSTS & DSTS_ENUMSPD)
    {
        default:
        /*
        case DSTS_ENUMSPD_LOW:
            m_enumeratedSpeed = Usb::LowSpeed;
            break;
        */

        case DSTS_ENUMSPD_FULL:
            m_enumeratedSpeed = Usb::FullSpeed;
            break;

        case DSTS_ENUMSPD_HIGH:
            m_enumeratedSpeed = Usb::HighSpeed;
            break;
    }

    // Inform the higher-level driver about the completion of the enumeration.
    derived().deviceEnumerationCompletedHandler();

    // Clear the global IN NAK flag.
    traits::regs()->DCTL |= DCTL_CGINAK;
    // Clear the global OUT NAK flag.
    traits::regs()->DCTL |= DCTL_CGONAK;

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_ENUMDNE;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleIncompleteIsochronousInTransfer()
{
    derived().deviceIncompleteIsochronousInHandler();

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_INCOMPISOIN;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleIncompleteIsochronousOutTransfer()
{
    derived().deviceIncompleteIsochronousOutHandler();

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_INCOMPISOOUT;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleInEndpointInterrupt()
{
    // Read the endpoint interrupt flags. Ignore those endpoints,
    // whose interrupt is masked.
    std::uint32_t daint = traits::regs()->DAINT & traits::regs()->DAINTMSK;
    // Only handle the IN endpoints.
    daint &= 0x0000FFFF;

    for (std::uint8_t endpointIdx = 0; daint != 0; ++endpointIdx)
    {
        std::uint32_t flag = std::uint32_t(1) << endpointIdx;
        if ((daint & flag) == 0)
            continue;
        daint &= ~flag;

        // Read the interrupt status of the IN endpoint. Note that the mask bit
        // for the TXFE flag is not located in the shared DIEPMSK register but
        // in the register DIEPEMPMSK because every endpoint has its own flag.
        std::uint32_t diepint = traits::DIEPINTx(endpointIdx);
        if (traits::regs()->DIEPEMPMSK & (std::uint32_t(1) << endpointIdx))
            diepint &= traits::regs()->DIEPMSK | DIEPINTx_TXFE;
        else
            diepint &= traits::regs()->DIEPMSK;

        if (diepint & DIEPINTx_XFRC)
        {
            // Clear the interrupt flag.
            traits::DIEPINTx(endpointIdx) = DIEPINTx_XFRC;

            // We do not react on an empty FIFO any longer.
            traits::regs()->DIEPEMPMSK &= ~(std::uint32_t(1) << endpointIdx);

            EndpointData& endpoint = m_inEndpoints[endpointIdx];
            // If the DMA is enabled, we have to advance the transaction buffer
            // pointer. In slave mode, this is already done when filling the
            // FIFO in fillTransmitFifo(), which is called when the
            // DIEPINTx_TXFE flag is set.
            if (traits::has_dma)
            {
                // The number of remaining bytes can be found in DIEPTSIZx.
                std::uint32_t numBytesSent
                        = endpoint.numBytesToTransceive
                          - (traits::DIEPTSIZx(endpointIdx) & DIEPTSIZx_XFRSIZ);
                if (numBytesSent > endpoint.numRemainingBytes)
                    numBytesSent = endpoint.numRemainingBytes;
                endpoint.buffer += numBytesSent;
                endpoint.numRemainingBytes -= numBytesSent;
            }

            // Inform the higher-level driver about the completion of the
            // transaction.
            derived().deviceInTransactionCompletedHandler(
                        endpointIdx,
                        endpoint.numBytesToTransceive
                          - endpoint.numRemainingBytes);
        }
        if (diepint & DIEPINTx_EPDISD)
        {
            //! \todo Handle this case.
            // Clear the interrupt flag.
            traits::DIEPINTx(endpointIdx) = DIEPINTx_EPDISD;
        }
        if (diepint & DIEPINTx_TOC)
        {
            //! \todo Handle this case
            // Clear the interrupt flag.
            traits::DIEPINTx(endpointIdx) = DIEPINTx_TOC;
        }
        if (diepint & DIEPINTx_TXFE)
        {
            // The transfer FIFO is empty and we can copy new packets into it.
            // Note: This interrupt flag is read-only, so there is no need to
            // reset it.
            if (!traits::has_dma)
                fillTransmitFifo(endpointIdx);
            // If all data has been written to the FIFO, we must disable the
            // FIFO-empty interrupt.
            EndpointData& endpoint = m_inEndpoints[endpointIdx];
            if (endpoint.numRemainingBytes == 0)
            {
                traits::regs()->DIEPEMPMSK &= ~(std::uint32_t(1) << endpointIdx);
            }
        }
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleOutEndpointInterrupt()
{
    // Read the endpoint interrupt flags. Ignore those endpoints,
    // whose interrupt is masked.
    std::uint32_t daint = traits::regs()->DAINT & traits::regs()->DAINTMSK;
    // Only handle the OUT endpoints.
    daint &= 0xFFFF0000;

    for (std::uint8_t endpointIdx = 0; daint != 0; ++endpointIdx)
    {
        std::uint32_t flag = std::uint32_t(1) << (endpointIdx + 16);
        if ((daint & flag) == 0)
            continue;
        daint &= ~flag;

        std::uint32_t doepint = traits::DOEPINTx(endpointIdx)
                                & traits::regs()->DOEPMSK;
        if (doepint & DOEPINTx_XFRC)
        {
            // The transfer has been completed.

            // Clear the interrupt flag.
            traits::DOEPINTx(endpointIdx) = DOEPINTx_XFRC;

            EndpointData& endpoint = m_outEndpoints[endpointIdx];
            if (traits::has_dma)
            {
                // If the DMA is enabled, we have to update the buffer pointer and
                // the number of bytes, which have been received. In slave mode this
                // is done when popping the receive FIFO in
                // handleRxFifoNonEmptyInterrupt().

                // The number of remaining bytes can be found in DOEPTSIZx.
                std::uint32_t numBytesReceived
                        = endpoint.numBytesToTransceive
                          - (traits::DOEPTSIZx(endpointIdx) & DOEPTSIZx_XFRSIZ);
                if (numBytesReceived > endpoint.numRemainingBytes)
                    numBytesReceived = endpoint.numRemainingBytes;

                endpoint.buffer += numBytesReceived;
                endpoint.numRemainingBytes -= numBytesReceived;
            }

            // Notify the higher-level driver about the completion of the transfer.
            derived().deviceOutTransactionCompletedHandler(
                        endpointIdx,
                        endpoint.numBytesToTransceive
                            - endpoint.numRemainingBytes);
        }
        if (doepint & DOEPINTx_EPDISD)
        {
            // The endpoint has been disabled.

            //! \todo Implement this case.
            // Clear the interrupt flag.
            traits::DOEPINTx(endpointIdx) = DOEPINTx_EPDISD;
        }
        if (doepint & DOEPINTx_STUP)
        {
            // The SETUP phase is done (for a control OUT endpoint).

            // Clear the interrupt flag.
            traits::DOEPINTx(endpointIdx) = DOEPINTx_STUP;

            if (endpointIdx != 0)
                continue;

            std::uint32_t numRemainingSetupPackets
                    = (traits::DOEPTSIZx(endpointIdx) & DOEPTSIZx_STUPCNT) >> 29;

            // Now inform the higher-level driver.
            // Note: In slave mode, we have already copied the SETUP packet
            // into the destination buffer in handleRxFifoNonEmptyInterrupt().
            // In DMA mode this has happened autonomously by the DMA.
            derived().deviceSetupPacketHandler(endpointIdx,
                                               3 - numRemainingSetupPackets);
        }
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleOtgProtocolEvent()
{
    std::uint32_t gotgint = traits::regs()->GOTGINT;

    // We only react on the "Session end detected" interrupt which is asserted
    // when Vbus < 0.8V.
    if (gotgint & GOTGINT_SEDET)
    {
        derived().deviceDisconnectHandler();
    }

    // Clear the interrupt flags. Note: The flag OTGINT in GINTSTS is cleared,
    // if all flags in GOTGINT are cleared.
    traits::regs()->GOTGINT = gotgint;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleResume()
{
    if (options::disable_clock_when_suspended)
    {
        // Re-enable the PHY clock and un-gate the USB core clock.
        traits::regs()->PCGCCTL &= ~(PCGCCTL_STPPCLK | PCGCCTL_GATEHCLK);
    }
    // Clear the remote wakeup signalling.
    traits::regs()->DCTL &= ~DCTL_RWUSIG;

    // Notify the higher-level driver about the resume event.
    derived().deviceResumeHandler();

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_WKUINT;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleRxFifoNonEmptyInterrupt()
{
    enum PacketStatus
    {
        GlobalOutNak              = 0x00020000,
        OutDataPacketReceived     = 0x00040000,
        OutTransferCompleted      = 0x00060000,
        SetupTransactionCompleted = 0x00080000,
        SetupDataPacketReceived   = 0x000C0000
    };

    // The packet read sequence is according to RM0090, rev 5, p. 1331.
    // - Read the Receive Status Pop register (GRXSTSP).
    // - Mask the RXFLVL interrupt in by clearing RXFLVLM in GINTMSK.
    // - Pop the packet data from the receive FIFO.

    // Mask the RXFLVL interrupt.
    traits::regs()->GINTMSK &= ~GINTMSK_RXFLVLM;
    // Pop the top-most word from the receive FIFO.
    std::uint32_t status = traits::regs()->GRXSTSP;

    // Decode the meaning of the FIFO entry. We only need to deal with those
    // entries which carry additional data (data OUT packet received and SETUP
    // packet received).

    switch (status & GRXSTSP_PKTSTS)
    {
        //! \todo Fuse these cases
        case OutDataPacketReceived:
        {
            if (status & GRXSTSP_BCNT)
            {
                std::uint8_t endpointNumber = status & GRXSTSP_EPNUM;
                std::uint32_t byteCount = (status & GRXSTSP_BCNT) >> 4;
                EndpointData& endpoint = m_outEndpoints[endpointNumber];
                readReceiveFifo(byteCount, endpoint.buffer,
                                endpoint.numRemainingBytes);
                if (byteCount > endpoint.numRemainingBytes)
                    byteCount = endpoint.numRemainingBytes;
                endpoint.buffer += byteCount;
                endpoint.numRemainingBytes -= byteCount;
            }
        } break;

        case SetupDataPacketReceived:
        {
            // Copy the SETUP packet into the buffer.
            std::uint8_t endpointNumber = status & GRXSTSP_EPNUM;
            EndpointData& endpoint = m_outEndpoints[endpointNumber];
            readReceiveFifo(8, endpoint.buffer, endpoint.numRemainingBytes);
            std::uint32_t byteCount = 8;
            if (byteCount > endpoint.numRemainingBytes)
                byteCount = endpoint.numRemainingBytes;
            endpoint.buffer += byteCount;
            endpoint.numRemainingBytes -= byteCount;

            // Note: We do not inform the higher-level driver about the SETUP
            // packet right now, because it might be that we receive up to
            // three SETUP packets back-to-back (per the USB 2.0 specification).
            // So we wait for the SETUP done interrupt---it is generated by the
            // USB core upon the completion of the STATUS stage---and inform the
            // higher-level driver from the handleOutEndpointInterrupt() method.
        } break;

        default:
            break;
    }

    // Unmask the RXFLVL interrupt.
    traits::regs()->GINTMSK |= GINTMSK_RXFLVLM;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleSessionRequest()
{
    derived().deviceConnectHandler();
    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_SRQINT;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleStartOfFrame()
{
    derived().deviceStartOfFrameHandler();
    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_SOF;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleSuspend()
{
    // Notify the higher-level driver about the suspend event.
    derived().deviceSuspendHandler();

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_USBSUSP;

    if (options::disable_clock_when_suspended
        && (traits::regs()->DSTS & DSTS_SUSPSTS) != 0)
    {
        // Switch off the clock of the PHY.
        traits::regs()->PCGCCTL |= PCGCCTL_STPPCLK;
        // Gate the HCLK.
        traits::regs()->PCGCCTL |= PCGCCTL_GATEHCLK;
        // Enter the sleep mode after we leave the interrupt handler.
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::handleUsbResetInterrupt()
{
    // Clear the remote wakeup signalling.
    traits::regs()->DCTL &= ~DCTL_RWUSIG;

    // Flush the transmit FIFO of endpoint IN 0.
    flushTransmitFifo(0);
    //! \todo flush the RX FIFO?

    // Next mask all endpoint interrupts.
    traits::regs()->DIEPMSK = 0;
    traits::regs()->DOEPMSK = 0;

    // Then disable all endpoints and clear their interrupt flags.
    for (unsigned idx = 0; idx < traits::max_num_in_endpoints; ++idx)
    {
        if (traits::DIEPCTLx(idx) & DIEPCTLx_EPENA)
            traits::DIEPCTLx(idx) = DIEPCTLx_EPDIS | DIEPCTLx_SNAK;
        traits::DIEPINTx(idx) = 0x1B;
    }

    for (unsigned idx = 0; idx < traits::max_num_out_endpoints; ++idx)
    {
        if (traits::DOEPCTLx(idx) & DOEPCTLx_EPENA)
            traits::DOEPCTLx(idx) = DOEPCTLx_EPDIS | DOEPCTLx_SNAK;
        traits::DOEPINTx(idx) = 0x1B;
    }

    // Unmask the interrupts for endpoint IN 0 and endpoint OUT 0.
    traits::regs()->DAINTMSK  = 0x00010001;

    // IN endpoints shall generate interrupts
    // - when the transfer has been completed,
    // - when the endpoint has been disabled or
    // - when a time-out occurs.
    traits::regs()->DIEPMSK = DIEPMSK_XFRCM | DIEPMSK_EPDM | DIEPMSK_TOM;
    //! \todo HS has another mask register (DINEP1MSK)

    // OUT endpoints shall generate interrupts
    // - when the transfer has been completed,
    // - when the endpoint has been disabled or
    // - when the SETUP phase is done.
    traits::regs()->DOEPMSK = DOEPMSK_XFRCM | DOEPMSK_EPDM | DOEPMSK_STUPM;
    //! \todo HS has another mask register (DOUTEP1MSK)

    // Zero the device address.
    setDeviceAddress(0);

    // Notify the higher-level driver about the USB reset.
    derived().deviceUsbResetHandler();

    // Clear the interrupt flag.
    traits::regs()->GINTSTS = GINTSTS_USBRST;
}

// ----=====================================================================----
//     Private methods
// ----=====================================================================----

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::initEndpoints()
{
    for (unsigned idx = 0; idx < traits::max_num_in_endpoints; ++idx)
    {
        EndpointData& endpoint = m_inEndpoints[idx];
        endpoint.setAddress(0x80 | idx);
        endpoint.setStallFlag(false);
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }

    for (unsigned idx = 0; idx < traits::max_num_out_endpoints; ++idx)
    {
        EndpointData& endpoint = m_outEndpoints[idx];
        endpoint.setAddress(idx);
        endpoint.setStallFlag(false);
        endpoint.buffer = 0;
        endpoint.numBytesToTransceive = 0;
        endpoint.numRemainingBytes = 0;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::initCore()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    //! \todo Make use of the pin-lib here.

    // ID (PA10)
    GPIOA->MODER    =  (GPIOA->MODER  & ~(3 << 20)) | (2  << 20);
    GPIOA->OTYPER  &= ~(1 << 10);
    GPIOA->AFR[1]   =  (GPIOA->AFR[1] & ~(15 << 8)) | (10 <<  8);
    GPIOA->OSPEEDR |=  (3 << 20);
    GPIOA->PUPDR   &= ~(3 << 20);

    // DM (PA11)
    GPIOA->MODER    =  (GPIOA->MODER  & ~(3  << 22)) | (2  << 22);
    GPIOA->OTYPER  &= ~(1 << 11);
    GPIOA->AFR[1]   =  (GPIOA->AFR[1] & ~(15 << 12)) | (10 << 12);
    GPIOA->OSPEEDR |=  (3 << 22);
    GPIOA->PUPDR   &= ~(3 << 22);

    // DP (PA12)
    GPIOA->MODER    =  (GPIOA->MODER  & ~(3  << 24)) | (2  << 24);
    GPIOA->OTYPER  &= ~(1 << 12);
    GPIOA->AFR[1]   =  (GPIOA->AFR[1] & ~(15 << 16)) | (10 << 16);
    GPIOA->OSPEEDR |=  (3 << 24);
    GPIOA->PUPDR   &= ~(3 << 24);

    //! \todo Vbus pin

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    RCC->AHB2RSTR |=  RCC_AHB2RSTR_OTGFSRST;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGFSRST;

    // Mask all interrupts.
    traits::regs()->GAHBCFG &= ~GAHBCFG_GINT;

    // Select the embedded full-speed serial transceiver and reset the USB core.
    traits::regs()->GUSBCFG |= GUSBCFG_PHSEL;
    // In order to reset the core, we have to wait until the master AHB is
    // in the idle state.
    while ((traits::regs()->GRSTCTL & GRSTCTL_AHBIDL) == 0); //! \todo Add a timeout
    // Initiate a USB core soft reset and wait until the core clears the bit.
    traits::regs()->GRSTCTL |= GRSTCTL_CSRST;
    while (traits::regs()->GRSTCTL & GRSTCTL_CSRST); //! \todo Add a timeout
    // Wait for at least 3 PHY clocks.
    weos::this_thread::sleep_for(weos::chrono::microseconds(3));

    // Set the turnaround time to 9 PHY cycles.
    //! \todo Needs to be adjusted for high-speed
    traits::regs()->GUSBCFG = (traits::regs()->GUSBCFG & ~GUSBCFG_TRDT)
                              | (std::uint32_t(9) << 10);
    // Force device mode. It takes up to 25 ms until this change takes effect.
    traits::regs()->GUSBCFG |= GUSBCFG_FDMOD;
    weos::this_thread::sleep_for(weos::chrono::milliseconds(50));

    // Enable the PHY clock and un-gate the USB core clock.
    traits::regs()->PCGCCTL = 0;

    // Set the device configuration:
    // - Generate the end of periodic micro-frame interrupt at 80% of the frame
    //   interval.
    // - Set the device speed. Note that this is not necessarily the speed at
    //   which the device will operate since this is only fixed during the USB
    //   speed enumeration.
    traits::regs()->DCFG = DCFG_PFIVL_80 | DCFG_DSPD_FULL;

    std::uint32_t gccfg = 0;
    // Leave the power-down state.
    gccfg |= GCCFG_PWRDWN;
    // Enable sensing the Vbus for "A" and "B" devices.
    gccfg |= GCCFG_VBUSASEN | GCCFG_VBUSBSEN;
    // Monitoring the Vbus is optional.
    if (!options::enable_vbus_monitoring)
        gccfg |= GCCFG_NOVBUSSENS;
    //! \todo: Have an option to enable this: gccfg |= GCCFG_SOFOUTEN;
    traits::regs()->GCCFG = gccfg;

    // Set up the FIFOs for reception and transmission.
    std::uint32_t start = 0;
    std::uint32_t size = (options::rx_fifo_size + 3) / 4;
    traits::regs()->GRXFSIZ = size;
    start += size;
    size = (options::tx0_fifo_size + 3) / 4;
    traits::regs()->DIEPTXF0 = (size << 16) | start;
    start += size;
    size = (options::tx1_fifo_size + 3) / 4;
    traits::regs()->DIEPTXF1 = (size << 16) | start;
    start += size;
    size = (options::tx2_fifo_size + 3) / 4;
    traits::regs()->DIEPTXF2 = (size << 16) | start;
    start += size;
    size = (options::tx3_fifo_size + 3) / 4;
    traits::regs()->DIEPTXF3 = (size << 16) | start;

    // Initialize the interrupt masks.
    traits::regs()->DIEPMSK = 0;
    traits::regs()->DOEPMSK = 0;
    traits::regs()->GINTMSK =   GINTMSK_SOFM
                              | GINTMSK_RXFLVLM
                              | GINTMSK_USBSUSPM
                              | GINTMSK_USBRSTM
                              | GINTMSK_ENUMDNEM
                              | GINTMSK_IEPINTM
                              | GINTMSK_OEPINTM
                              | GINTMSK_IISOIXFRM
                              | GINTMSK_IISOOXFRM
                              | GINTMSK_WUIM;
    if (options::enable_vbus_monitoring)
    {
        traits::regs()->GINTMSK |= GINTMSK_OTGINTM | GINTMSK_SRQIM;
    }

    // Re-enable interrupts.
    //! \todo If GAHBCFG_TXFELVL was not set, an interrupt would be generated,
    //! when a FIFO is half empty. Could be helpful!
    traits::regs()->GAHBCFG |= GAHBCFG_GINT | GAHBCFG_TXFELVL;
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::fillTransmitFifo(
        std::uint8_t inEndpointNumber)
{
    EndpointData& endpoint = m_inEndpoints[inEndpointNumber];
    while (1)
    {
        std::uint32_t bufferSize = endpoint.numRemainingBytes;
        if (bufferSize == 0)
            return;
        if (bufferSize > endpoint.maxPacketSize())
            bufferSize = endpoint.maxPacketSize();
        std::uint32_t numWords = (bufferSize + 3) / 4;

        // Check the available FIFO space.
        if ((traits::DTXFSTSx(inEndpointNumber) & DTXFSTSx_INEPTFSAV) < numWords)
            return;

        volatile std::uint32_t& fifo = traits::TXFIFOx(inEndpointNumber);
        std::uint8_t* src = endpoint.buffer;
        while (numWords--)
        {
            fifo = *reinterpret_cast<__packed std::uint32_t*>(src);
            src += 4;
        }
        endpoint.buffer += bufferSize;
        endpoint.numRemainingBytes -= bufferSize;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::flushTransmitFifo(
        std::uint8_t inEndpointNumber)
{
    traits::regs()->GRSTCTL =   (std::uint32_t(inEndpointNumber) << 6)
                              | GRSTCTL_TXFFLSH;
    // The TXFFLSH bit is cleared by the USB core at the end of the flush
    // operation (RM0090, rev 5, p. 1342).
    for (std::uint8_t tries = 64; tries != 0; --tries)
    {
        if ((traits::regs()->GRSTCTL & GRSTCTL_TXFFLSH) == 0)
            break;
    }
}

template <typename TUsbSelector, typename TDerived, typename TOptions>
void UsbDeviceCoreDriver<TUsbSelector, TDerived, TOptions>::readReceiveFifo(
        std::uint32_t fifoSize, void* buffer, std::uint32_t bufferSize)
{
    // Compute the number of words which have to be popped from the FIFO.
    // The data is stored in a 32-bit FIFO which is why we have to round the
    // number of bytes to the next multiple of 4.
    std::uint32_t numWordsInFifo = (fifoSize + 3) / 4;
    // First perform a word-wise copy into the output buffer.
    std::uint32_t numWordsInBuffer = bufferSize / 4;
    if (numWordsInBuffer > numWordsInFifo)
        numWordsInBuffer = numWordsInFifo;
    numWordsInFifo -= numWordsInBuffer;

    std::uint8_t* dest = static_cast<std::uint8_t*>(buffer);
    while (numWordsInBuffer--)
    {
        *reinterpret_cast<__packed std::uint32_t*>(dest) = traits::RXFIFO();
        dest += 4;
    }

    // Determine the number of words which are left in the FIFO. Return to the
    // caller if nothing is left.
    if (numWordsInFifo == 0)
        return;

    // There is something left in the FIFO. If the bufferSize is not a multiple
    // of 4, we can still store 1 to 3 bytes in the buffer.
    std::uint32_t datum = traits::RXFIFO();
    --numWordsInFifo;
    switch (bufferSize % 4)
    {
        case 3:
            *dest++ = datum;
            datum >>= 8;
        case 2:
            *dest++ = datum;
            datum >>= 8;
        case 1:
            *dest++ = datum;
        default:
            // Finally, pop off any value which is left in the FIFO.
            while (numWordsInFifo--)
            {
                datum = traits::RXFIFO();
            }
    }
}

} // namespace detail
} // namespace m15

#endif // M15_STM32F4XX_USBDEVICECOREDRIVER_HPP
