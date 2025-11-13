package com.hamosad.lib.components.sensors

import com.hamosad.lib.math.Length
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class HaDistanceSensor(hardwareMap: HardwareMap, name: String) {
    private val sensor: DistanceSensor = hardwareMap.get(DistanceSensor::class.java, name)
    val distance: Length get() = Length.fromCentimeters(sensor.getDistance(DistanceUnit.CM))
}