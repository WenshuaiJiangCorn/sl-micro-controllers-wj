/**
 * @file
 * @brief The header-only file for the TorqueModule class. This class allows interfacing with a torque sensor to
 * monitor the torque applied to the connected object.
 *
 * @attention Since most torque sensors output a differential signal to code for the direction of the torque, this
 * class is designed to work with an AD620 microvolt amplifier to straighten and amplify the output of the torque
 * sensor.
 *
 * @section trq_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_TORQUE_MODULE_H
#define AXMC_TORQUE_MODULE_H

#include <Arduino.h>
#include <axmc_shared_assets.h>
#include <digitalWriteFast.h>
#include <module.h>

/**
 * @brief Monitors the signal sent by the torque sensor through an AD620 microvolt amplifier.
 *
 * This module is specifically designed to work with fine-resolution torque sensors that output differential signals
 * in the millivolt or microvolt range that is picked up and amplified by the AD620 amplifier. This module works by
 * monitoring an analog input pin for the signal from the AD620 amplifier expected to be zeroed ~ 1.6 V. This way,
 * torque in the CCW direction is detected as a positive deflection from baseline, and torque in the CW direction
 * is detected as a negative deflection from baseline. The class can be flexibly configured to report torque in either
 * direction and to only report significant torque changes.
 *
 * @tparam kPin the analog pin whose state will be monitored to detect torque's direction and magnitude.
 * @tparam kBaseline the value, in ADC units, for the analog signal sent by the sensor when it detects no torque. This
 * value depends on the zero-point calibration of the AD620 amplifier, and it is used to shift differential bipolar
 * signal of the torque sensor to use a positive integer scale. Signals that are originally above the baseline are
 * interpreted as CounterClockWise (CCW) torque, and signals below the baseline are interpreted as ClockWise (CW)
 * torque. When the class reports torque to the PC, it further adjusts the torque value to always be between 0 and
 * Baseline, reporting the direction via the event-code of the message.
 * @tparam kInvertDirection if true, inverts the direction the torque returned by the sensor. By default, the sensor
 * interprets torque in the CCW direction as positive and torque in the CW direction as negative. This flag allows
 * reversing this relationship, which may be helpful, depending on how the sensor is mounted and wired.
 */
template <const uint8_t kPin, const uint16_t kBaseline, const bool kInvertDirection = false>
class TorqueModule final : public Module
{
        // Also ensures that torque sensor pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for TorqueModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kCCWTorque = 51,  ///< The sensor detects torque in the CCW direction.
            kCWTorque  = 52,  ///< The sensor detects torque in the CW direction.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if necessary informs the PC of any changes.
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        TorqueModule(
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
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin to Input mode.
            pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW        = true;
            _custom_parameters.report_CW         = true;
            _custom_parameters.signal_threshold  = 100;
            _custom_parameters.delta_threshold   = 70;
            _custom_parameters.average_pool_size = 5;

            // Notifies the PC about the initial sensor state. Primarily, this is needed to support data source
            // time-alignment during post-processing.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque),  // Direction is not relevant for 0 value
                axmc_communication_assets::kPrototypes::kOneUint16,
                0
            );

            return true;
        }

        ~TorqueModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                bool report_CCW           = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW            = true;  ///< Determines whether to report changes in the CCW direction.
                uint16_t signal_threshold = 100;   ///< The minimum deviation from the baseline to be reported to PC.
                uint16_t delta_threshold  = 70;  ///< The minimum signal difference between checks to be reported to PC.
                uint8_t average_pool_size = 5;   ///< The number of readouts to average into pin state value.
        } PACKED_STRUCT _custom_parameters;

        /// Checks the signal received by the input pin and, if necessary, reports it to the PC.
        void CheckState()
        {
            // Stores the previous readout of the analog pin. This is used to limit the number of messages sent to the
            // PC by only reporting significant changes of the pin state (signal). The level that constitutes
            // significant change can be adjusted through the custom_parameters structure. The value is
            // initialized to the baseline that denotes zero-torque readouts.
            static uint16_t previous_readout = kBaseline;  // NOLINT(*-dynamic-static-initializers)

            // Tracks whether the previous message sent to the PC included a zero torque value. This is used to
            // eliminate repeated reporting of sub-threshold values pulled down to 0 to the PC. In turn, this conserves
            // communication bandwidth
            static bool previous_zero = false;

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value. Note, since we statically configure the controller to use 10-14 bit ADC resolution,
            // this value should not use the full range of the 16-bit unit variable.
            uint16_t signal = AnalogRead(kPin, _custom_parameters.average_pool_size);

            // Calculates the absolute difference between the current signal and the previous readout. This is used
            // to ensure only significant signal changes are reported to the PC. Note, although we are casting both to
            // int32 to support the delta calculation, the resultant delta will always be within the unit_16 range.
            // Therefore, it is fine to cast it back to uint16 to avoid unnecessary future casting in the if statements.
            const uint16_t delta = abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout));

            // Prevents reporting signals that are not significantly different from the previous readout value.
            if (delta <= _custom_parameters.delta_threshold)
            {
                CompleteCommand();
                return;
            }

            previous_readout = signal;  // Overwrites the previous readout with the current signal.

            // Determines the direction of the signal. Signals above the baseline are interpreted as CCW, signals below
            // baseline as CW. Also rescales the signal as necessary for its value to always mean the same things: 0
            // is no torque, baseline is maximum torque, regardless of direction.
            bool cw;
            if (signal > kBaseline)
            {
                // For CCW signals, subtracts the baseline from the signal to ensure the signal is always scaled between
                // 0 and baseline. CCW torque increases as the level of the signal increases. Therefore, the larger
                // the signal, the higher the torque.
                cw     = false ^ kInvertDirection;
                signal = signal - kBaseline;
            }
            else if (signal < kBaseline)
            {
                // For CW signals, subtracts the signal from the baseline. CW torque increases as the level of
                // the signal decreases. Therefore, the smaller the signal, the higher the torque. This rescaling
                // 'shifts' the signal to increase proportionally to CW torque, from 0 to baseline.
                cw     = true ^ kInvertDirection;
                signal = kBaseline - signal;
            }
            else
            {
                // If the signal is 0, the direction does not really matter
                cw     = false;  // Uses CCW for zero-torque reporting
                signal = 0;
            }

            // If the signal is below the threshold, pulls it to 0 and notifies the PC. The torque direction in this
            // case does not matter
            if (signal < _custom_parameters.signal_threshold)
            {
                // Ensures sub-threshold values are only sent to the PC once if multiple zero-values occur in succession
                if (!previous_zero)
                {
                    SendData(
                        static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque),  // Uses CCW for 0-torque reporting
                        axmc_communication_assets::kPrototypes::kOneUint16,
                        0
                    );
                    previous_zero = true;
                }
            }
            else
            {
                // Sends the detected signal to the PC using the event-code to code for the direction and the
                // signal value to provide the absolute directional torque value in raw ADC units. Only sends the data
                // if the class is configured to report changes in that direction
                if (!cw && _custom_parameters.report_CCW)
                {
                    SendData(
                        static_cast<uint8_t>(kCustomStatusCodes::kCCWTorque),
                        axmc_communication_assets::kPrototypes::kOneUint16,
                        signal
                    );
                }
                else if (cw && _custom_parameters.report_CW)
                {
                    SendData(
                        static_cast<uint8_t>(kCustomStatusCodes::kCWTorque),
                        axmc_communication_assets::kPrototypes::kOneUint16,
                        signal
                    );
                }

                previous_zero = false;
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_TORQUE_MODULE_H
