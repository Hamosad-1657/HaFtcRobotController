package com.hamosad.lib.components.sensors

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

class HaTouchSensor(hardwareMap: HardwareMap, name: String) {
    private val sensor: TouchSensor = hardwareMap.get(TouchSensor::class.java, name)
    val isTouched get() = sensor.isPressed
}