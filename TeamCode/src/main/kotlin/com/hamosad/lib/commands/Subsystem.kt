package com.hamosad.lib.commands

import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Subsystem(var hardwareMap: HardwareMap) {
    /** If you want to override it, make sure to call super.restartSubsystem() in the start for the subsystem to function correctly. */
    fun restartSubsystem(newHardwareMap: HardwareMap) {
        hardwareMap = newHardwareMap
    }
    abstract var defaultCommand: Command?
    abstract fun periodic()
}