package com.hamosad.lib.components.sensors

import com.hamosad.lib.math.Length
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class HaColorSensor(hardwareMap: HardwareMap, name: String) {
    val colorSensor: RevColorSensorV3 = hardwareMap.get(RevColorSensorV3::class.java, name)

    init {
        colorSensor.initialize()
        colorSensor.enableLed(true)
    }

    val maxValue = 4095.0
    val red get() = (colorSensor.red() / maxValue * 255.0).toInt()
    val blue get() = (colorSensor.blue() / maxValue * 255.0).toInt()
    val green get() = (colorSensor.green() / maxValue * 255.0).toInt()

    val color get() = Color(red, green, blue)

    val distance: Length get() = Length.fromCentimeters(colorSensor.getDistance(DistanceUnit.CM))

    fun isInColorRange(color: Color, deviation: Int): Boolean {
        return red in (color.red - deviation)..(color.red + deviation) &&
                blue in (color.blue -deviation)..(color.blue + deviation) &&
                green in (color.green - deviation)..(color.green + deviation)
    }

    fun enableLed() {
        colorSensor.enableLed(true)
    }

    fun disableLed() {
        colorSensor.enableLed(false)
    }
}

class Color(val red: Int, val green: Int, val blue: Int) {
    companion object {
        val red = Color(255, 0 ,0)
        val green = Color(0, 255, 0)
        val blue = Color(0, 0, 255)
        val yellow = Color(255,  255, 0 )
        val white = Color(255, 255, 255)
        val purple = Color(255, 0, 255)
        val orange = Color(255, 108, 0)
        val black = Color(0, 0,0)
    }
}