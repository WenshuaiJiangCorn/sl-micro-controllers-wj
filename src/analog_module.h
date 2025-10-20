#ifndef ANALOG_MODULE_WJ
#define ANALOG_MODULE_WJ

#include <Arduino.h>
#include <axmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <module.h>

template <const uint8_t kPin>
class AnalogModule final : public Module 
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for AnalogModule instance."
        );

    public:
    
};
#endif  //ANALOG_MODULE_WJ
