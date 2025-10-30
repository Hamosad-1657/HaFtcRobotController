package com.hamosad.lib.commands

import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Subsystem(var hardwareMap: HardwareMap) {


    fun restartSubsystem(newHardwareMap: HardwareMap) {
        hardwareMap = newHardwareMap
    }
    abstract var defaultCommand: Command?
    abstract fun periodic()
}