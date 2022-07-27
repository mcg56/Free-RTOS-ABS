#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"


/**
 * @brief Initialisation functions for the clock
 * @arg None
 * @return None
 */
initClock (void)
{
    // Set the clock rate to 80 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}
