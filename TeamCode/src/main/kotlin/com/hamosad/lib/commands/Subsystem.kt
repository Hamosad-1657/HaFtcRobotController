package com.hamosad.lib.commands

import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Subsystem(val hardwareMap: HardwareMap) {
    abstract var defaultCommand: Command?
    abstract fun periodic()
}