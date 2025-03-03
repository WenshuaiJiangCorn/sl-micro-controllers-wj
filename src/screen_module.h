/**
* @file
 *
 * @brief The header-only file for the ScreenModule class. This class allows interfacing with the screens used in the
 * Sun lab's Virtual Reality system. Specifically, it operates a logic gate that turns the screen displays on and off
 * without interfering with the display configuration of the host PC by directly manipulating the HDMI-translator card
 * hardware.
 *
 * @section scr_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef SCREEN_MODULE_H
#define SCREEN_MODULE_H

#include <Arduino.h>
#include <axmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Pulses the connected FET gate to simulate pressing the power button on the VR screen control panel.
 *
 * This class is specifically designed to interface with a one-channel FET gate hard-soldered to the circuitry of the
 * VR screen control panel. Upon receiving the Toggle command, this class pulses (opens and closes) the FET gate to
 * simulate the user pressing the power button. In turn, this changes the power state of the VR screen between ON and
 * OFF, without affecting the display configuration of the host PC.
 *
 * @tparam kLeftScreenPin the digital pin used to control the power state of the Left VR screen, from the animal's
 * perspective.
 * @tparam kCenterScreenPin the digital pin used to control the power state of the Center VR screen, from the animal's
 * perspective.
 * @tparam kRightScreenPin the digital pin used to control the power state of the Right VR screen, from the animal's
 * perspective.
 * @tparam kNormallyClosed determines the type of the FET gate relay used to control the power state of the VR screens.
 * Specifically, a NormallyClosed relay is closed (Off) when the logic signal to the relay is LOW and opened (On) when
 * the logic signal to the relay is HIGH.
 */
template <
    const uint8_t kLeftScreenPin,
    const uint8_t kCenterScreenPin,
    const uint8_t kRightScreenPin,
    const bool kNormallyClosed = true>
class ScreenModule final : public Module
{
        static_assert(
            kLeftScreenPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Left display screen pin for "
            "ScreenModule instance."
        );
        static_assert(
            kCenterScreenPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Center display screen pin for "
            "ScreenModule instance."
        );
        static_assert(
            kRightScreenPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Right display screen pin for "
            "ScreenModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kOutputLocked = 51,  ///< The output ttl pin is in a global locked state and cannot be toggled on or off.
            kOn           = 52,  ///< The relay is receiving a HIGH signal
            kOff          = 53,  ///< The relay is receiving a LOW signal
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kToggle = 1,  ///< Pulses the connected FET gate to simulate pressing the power button on the VR screen.
        };

        /// Initializes the class by subclassing the base Module class.
        ScreenModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const axmc_shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Extracts the received parameters into the _custom_parameters structure of the class. If extraction fails,
            // returns false. This instructs the Kernel to execute the necessary steps to send an error message to the
            // PC.
            return _communication.ExtractModuleParameters(_custom_parameters);
        }

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // Toggle screen state
                case kModuleCommands::kToggle: Toggle(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            pinModeFast(kLeftScreenPin, OUTPUT);
            pinModeFast(kCenterScreenPin, OUTPUT);
            pinModeFast(kRightScreenPin, OUTPUT);

            // Depending on the class configuration, ensures the logic gates are disabled at startup
            digitalWriteFast(kLeftScreenPin, kOff);
            digitalWriteFast(kCenterScreenPin, kOff);
            digitalWriteFast(kRightScreenPin, kOff);

            SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration = 1000000;  // 1000000 microseconds == 1 second.

            return true;
        }

        ~ScreenModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 1000000;  ///< The time, in microseconds, the pins output HIGH during pulses.
        } PACKED_STRUCT _custom_parameters;

        /// The value the output pins have to be set for the connected gate to be activated and allow the current to
        /// flow through the gate. Computing this value statically ensures that the class behaves the same way
        /// regardless of whether it is normally opened (NO) or closed (NC)
        static constexpr bool kOn = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// The value the output pins have to be set for the connected gate to be deactivated and stop allowing the
        /// current to flow through the gate.
        static constexpr bool kOff = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Sends a digital pulse through all output pins, using the preconfigured pulse_duration of microseconds.
        /// Supports non-blocking execution and respects the global ttl_lock state.
        void Toggle()
        {
            // Initializes the pulse
            if (execution_parameters.stage == 1)
            {
                // Sets all screen pins to ON state
                bool state = DigitalWrite(kLeftScreenPin, kOn, false);
                if (state) state = DigitalWrite(kCenterScreenPin, kOn, false);
                if (state) state = DigitalWrite(kRightScreenPin, kOn, false);
                if (state)
                {
                    // If all pins were set successfully, sends a confirmation message to the PC and advances the
                    // command stage.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOn));
                    AdvanceCommandStage();
                }
                else
                {
                    // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }
            }

            // Delays for the requested number of microseconds before inactivating the pulse.
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to the time of the last AdvanceCommandStage()
                // call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                AdvanceCommandStage();
            }

            // Inactivates the pulse
            if (execution_parameters.stage == 3)
            {
                // Sets all screen pins to OFF state
                bool state = DigitalWrite(kLeftScreenPin, kOff, false);
                if (state) state = DigitalWrite(kCenterScreenPin, kOff, false);
                if (state) state = DigitalWrite(kRightScreenPin, kOff, false);
                if (state)
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));
                    CompleteCommand();
                }
                else
                {
                    // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                }
            }
        }
};

#endif  //SCREEN_MODULE_H
