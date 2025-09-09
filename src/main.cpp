// To upload the code to each of the target microcontrollers, modify the target microcontroller name on line 26 and
// use platformio to compile and upload the project. When uploading the code, make sure only one
// Teensy 4.1 is connected to the host-PC at the same time.

// The microcontrollers assembled and configured as part of this project are designed to work with the Python
// interfaces available from the sl-experiment project: https://github.com/Sun-Lab-NBB/sl-experiment.
// See https://github.com/Sun-Lab-NBB/sl-micro-controllers for more details on assembling the hardware and
// installing the project.
// API documentation: https://sl-micro-controllers-api-docs.netlify.app/.
// Author: Ivan Kondratyev (Inkaros).

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
// WJ note: only ACTOR is currently used.

// Resolves microcontroller-specific module configuration and layout
// Use the same controller as for valve and lick

#include "valve_module.h"
#include "lick_module.h"

constexpr uint8_t kControllerID = 101;

ValveModule<6,  true, true, 255> reward_valve_1(5, 1, axmc_communication, DynamicRuntimeParameters);
ValveModule<19, true, true, 255> reward_valve_2(5, 2, axmc_communication, DynamicRuntimeParameters);

LickModule<3>  lick_sensor_1(4, 1, axmc_communication, DynamicRuntimeParameters);
LickModule<22> lick_sensor_2(4, 2, axmc_communication, DynamicRuntimeParameters);

Module* modules[] = {
    &reward_valve_1,
    &reward_valve_2,
    &lick_sensor_1,
    &lick_sensor_2
};

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, DynamicRuntimeParameters, modules);

void setup()
{
    Serial.begin(115200);  // The baudrate is ignored for teensy boards.

// Disables unused 3.3V pins hooked up to the 3-5 V voltage shifter. If this is not done, the shifter will output a
// HIGH signal from non-disabled pins.

/*
#ifdef ACTOR
    pinMode(33, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(34, OUTPUT);
    digitalWrite(36, LOW);
    pinMode(35, OUTPUT);
    digitalWrite(35, LOW);
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);

#endif
*/
    // Sets ADC resolution to 12 bits. Teensy boards can support up to 16 bits, but 12 often produces cleaner readouts.
    analogReadResolution(12);

    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.
}

void loop()
{
    axmc_kernel.RuntimeCycle();
}
