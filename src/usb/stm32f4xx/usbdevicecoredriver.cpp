#include <m15/usb/usbdevicecoredriver.hpp>

namespace m15
{
namespace detail
{

UsbCoreDriver* usb_core_traits<usb_full_speed_t>::coreDriver = 0;

} // namespace detail
} // namespace m15

// ----=====================================================================----
//     Interrupts
// ----=====================================================================----

extern "C" void OTG_FS_IRQHandler(void)
{
    typedef m15::detail::usb_core_traits<m15::usb_full_speed_t> traits;
    if (traits::coreDriver)
        traits::coreDriver->handleInterrupt();
}

//! \todo OTG_FS_WKUP_IRQHandler() is missing
