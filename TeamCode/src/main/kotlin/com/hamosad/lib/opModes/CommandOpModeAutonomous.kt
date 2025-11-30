package com.hamosad.lib.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.CommandScheduler
import com.hamosad.lib.commands.Subsystem
import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class CommandOpModeAutonomous: OpMode() {
    abstract var subsystemsToUse: List<Subsystem>

    var useTelemetry: Boolean = true

    /** Called once when OpMode init is pressed, before command scheduler initialization. */
    open fun disabledInit() {}

    /** Set your default commands here. */
    abstract fun configureDefaultCommands()

    /** Your autonomous command to be executed at start. */
    abstract fun getAutonomousCommand(): Command

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
    }

    // Called repeatedly after init is pressed
    final override fun init_loop() {
        val packet = TelemetryPacket()
        for (subsystem in subsystemsToUse) {
            subsystem.periodic()
            if (useTelemetry) subsystem.updateTelemetry(super.telemetry, packet)
        }
        super.telemetry.update()
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    // Called when start is pressed
    final override fun start() {
        CommandScheduler.initialize()
        CommandScheduler.scheduleCommand(getAutonomousCommand())
    }

    // Called repeatedly after start is pressed
    final override fun loop() {
        CommandScheduler.execute()
        if (useTelemetry) {
            val packet = TelemetryPacket()
            for (subsystem in subsystemsToUse) {
                subsystem.updateTelemetry(super.telemetry, packet)
            }
            super.telemetry.update()
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }

    // Called when stop is pressed
    final override fun stop() {
        CommandScheduler.reset()
    }
}