// Dependencies
#include <Arduino.h>
#include <axmc_shared_assets.h>
#include <communication.h>
#include <kernel.h>
#include <module.h>

// Initializes the shared microcontroller parameter structure. This structure is used by all microcontroller types.
axmc_shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters;

// Initializes the serial communication class.
Communication axmc_communication(Serial);  // NOLINT(*-interfaces-global-init)

// Defines the target microcontroller. Our VR system currently has 3 valid targets: ACTOR, SENSOR and ENCODER.
#define ACTOR

// Resolves microcontroller-specific module configuration and layout
#ifdef ACTOR
#include "break_module.h"
#include "ttl_module.h"
#include "valve_module.h"

constexpr uint8_t kControllerID = 101;
TTLModule<33, true, false> mesoscope_trigger(1, 1, axmc_communication, DynamicRuntimeParameters);
BreakModule<28, false, true> wheel_break(3, 1, axmc_communication, DynamicRuntimeParameters);
ValveModule<29, true, true> reward_valve(5, 1, axmc_communication, DynamicRuntimeParameters);
Module* modules[] = {&mesoscope_trigger, &wheel_break, &reward_valve};

#elif defined SENSOR
#include "lick_module.h"
#include "torque_module.h"
#include "ttl_module.h"

constexpr uint8_t kControllerID = 152;
TTLModule<34, false, false> mesoscope_frame(1, 1, axmc_communication, DynamicRuntimeParameters);
LickModule<40> lick_sensor(4, 1, axmc_communication, DynamicRuntimeParameters);
TorqueModule<41, 2048, true> torque_sensor(6, 1, axmc_communication, DynamicRuntimeParameters);
Module* modules[] = {&mesoscope_frame, &lick_sensor, &torque_sensor};

#elif defined ENCODER
#include "encoder_module.h"

constexpr uint8_t kControllerID = 203;
EncoderModule<33, 34, 35> wheel_encoder(2, 1, axmc_communication, DynamicRuntimeParameters);
Module *modules[] = {&wheel_encoder};
#else
static_assert(false, "Define one of the supported microcontroller targets (ACTOR, SENSOR, ENCODER).");
#endif

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, DynamicRuntimeParameters, modules);

void setup()
{
    Serial.begin(115200);  // The baudrate is ignored for teensy boards.

#ifdef ACTOR
    pinMode(34, OUTPUT);
    digitalWrite(34, LOW);
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);

#elif defined SENSOR
    pinMode(33, OUTPUT);
    digitalWrite(33, LOW);
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);

#elif defined ENCODER
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);
#endif

    // Sets ADC resolution to 12 bits. Teensies can support up to 16 bits, but 12 often produces cleaner readouts.
    analogReadResolution(12);

    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.
}

void loop()
{
    axmc_kernel.RuntimeCycle();
}
