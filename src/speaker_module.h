/**
 * @file
 *
 * @brief The header-only file for the SpeakerModule class. This class allows interfacing with the piezoelectric buzzer
 * speakers used in the Sun lab's Virtual Reality system. Specifically, it operates a FET gate to deliver direct voltage
 * to the piezoelectric buzzer that emits a tone signal. Note, the volume of the signal scales with the delivered
 * voltage and can reach 80-90 dB. This module is intended to be used together with the ValveModule to provide
 * auditory feedback whenever the valve is pulsed.
 *
 * @section spk_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef SPEAKER_MODULE_H
#define SPEAKER_MODULE_H

#include <Arduino.h>
#include <axmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Pulses the connected FET gate to deliver a brief tone via the piezoelectric buzzer module.
 *
 * This module is specifically designed to send digital signals that trigger Field-Effect-Transistor (FET) gated relay
 * hardware to deliver voltage that is converted into an audible tone by the piezoelectric buzzer.
 *
 * @tparam kPin the digital pin connected to the piezoelectric buzzer FET-gated relay.
 * @tparam kStartOff determines whether the piezoelectric buzzer is initially powered on (true) or off (false).
 */
template <const uint8_t kPin, const bool kStartOff = true>
class SpeakerModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for SpeakerModule instance."
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
            kSendPulse = 1,  ///< Delivers a brief auditory tone by sending a HIGH signal to the relay.
            kToggleOn  = 2,  ///< Sets the buzzer to deliver a continuous tone.
            kToggleOff = 3,  ///< Permanently silences the buzzer.
        };

        /// Initializes the class by subclassing the base Module class.
        SpeakerModule(
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
                // Pulse
                case kModuleCommands::kSendPulse: Pulse(); return true;
                // Open
                case kModuleCommands::kToggleOn: Open(); return true;
                // Close
                case kModuleCommands::kToggleOff: Close(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinModeFast(kPin, OUTPUT);

            // Based on the requested initial buzzer state, configures the buzzer to either be silent or deliver a
            // continuous tone
            if (kStartOff)
            {
                digitalWriteFast(kPin, LOW);  // Silences the buzzer.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));
            }
            else
            {
                digitalWriteFast(kPin, HIGH);  // Delivers a continuous tone.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOn));
            }

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration = 100000;  // Default tone duration is 100 ms.

            return true;
        }

        ~SpeakerModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 100000;  ///< The time, in microseconds, the pin outputs HIGH during pulses.
        } PACKED_STRUCT _custom_parameters;

        /// Cycles HIGH and LOW signals to deliver a brief tone via the buzzer.
        void Pulse()
        {
            // Initiates the tone.
            if (execution_parameters.stage == 1)
            {
                // Toggles the pin to send the high signal. If the pin is successfully activated, as indicated by the
                // DigitalWrite returning true, advances the command stage.
                if (DigitalWrite(kPin, HIGH, false))
                {
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

            // Delays for the tone duration
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to the time of the last AdvanceCommandStage()
                // call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                AdvanceCommandStage();
            }

            // Silences the tone
            if (execution_parameters.stage == 3)
            {
                // Once the pulse duration has passed, inactivates the pin by setting it to LOW signal. Finishes
                // command execution if inactivation is successful.
                if (DigitalWrite(kPin, LOW, false))
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

        /// Continuously delivers the tone.
        void Open()
        {
            // Sets the pin to HIGH signal and finishes command execution
            if (DigitalWrite(kPin, HIGH, false))
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOn));
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

        /// Permanently silences the tone.
        void Close()
        {
            // Sets the pin to LOW signal and finishes command execution
            if (DigitalWrite(kPin, LOW, false))
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOff));
                CompleteCommand();  // Finishes command execution
            }
            else
            {
                // If writing to actor pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }
};

#endif  //SPEAKER_MODULE_H
