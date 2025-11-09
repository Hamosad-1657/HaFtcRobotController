package com.hamosad.lib.commands

import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Subsystem() {
    var isInitialized: Boolean = false
        private set
    protected var hardwareMap: HardwareMap? = null

    /** If you want to override it, make sure to call super.restartSubsystem() in the start for the subsystem to function correctly.
     * Your hardware initialization should be done here.
     **/
    open fun init(newHardwareMap: HardwareMap) {
        hardwareMap = newHardwareMap
        isInitialized = true
    }
    var defaultCommand: Command? = null

    /** From the moment init is pressed, this function is called repeatedly until OpMode is stopped. */
    abstract fun periodic()

    // TODO: Telemetry
}