package com.hamosad.lib.opModes

import com.hamosad.lib.commands.CommandScheduler
import com.hamosad.lib.commands.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class CommandOpModeTeleop: OpMode() {
    abstract var subsystemsToUse: List<Subsystem>

    var useTelemetry: Boolean = true

    /** Called once when OpMode init is pressed, before command scheduler initialization. */
    open fun disabledInit() {}

    /** Set your bindings here.*/
    abstract fun configureBindings()

    /** Set your default commands here. */
    abstract fun configureDefaultCommands()

    // Called when init is pressed
    final override fun init() {
        disabledInit()

        for (subsystem in subsystemsToUse) {
            if (!subsystem.isInitialized) {
                subsystem.init(super.hardwareMap)
            }
            CommandScheduler.registerSubsystem(subsystem)
        }

        configureDefaultCommands()
        configureBindings()
    }

    // Called repeatedly after init is pressed
    final override fun init_loop() {
        for (subsystem in subsystemsToUse) {
            subsystem.periodic()
            if (useTelemetry) subsystem.updateTelemetry(super.telemetry)
        }
        if (useTelemetry) super.telemetry.update()
    }

    // Called when start is pressed
    final override fun start() {
        CommandScheduler.initialize()
    }

    // Called repeatedly after start is pressed
    final override fun loop() {
        CommandScheduler.execute()
        if (useTelemetry) {
            for (subsystem in subsystemsToUse) {
                subsystem.updateTelemetry(super.telemetry)
            }
            super.telemetry.update()
        }
    }

    // Called when stop is pressed
    final override fun stop() {
        CommandScheduler.reset()
    }
}