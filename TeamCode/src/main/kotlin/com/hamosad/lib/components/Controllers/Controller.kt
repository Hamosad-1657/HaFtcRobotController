package com.hamosad1657.lib.controllers

import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.continuousDeadband
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

/**
 * Automatically flips y value of joysticks and applies deadband
 */
class HaController(hardwareMap: HardwareMap, val deadband: Double, private val power: Int = 1, name: String) {
    val controller: Gamepad = hardwareMap.get(Gamepad::class.java, name)

    fun getLeftX(): Double {
        return continuousDeadband(controller.left_stick_x.toDouble(), deadband).powerProfile(power)
    }

    fun getLeftY(): Double {
        return -continuousDeadband(controller.left_stick_y.toDouble(), deadband).powerProfile(power)
    }

    fun getRightX(): Double {
        return continuousDeadband(controller.right_stick_x.toDouble(), deadband).powerProfile(power)
    }

    fun getRightY(): Double {
        return -continuousDeadband(controller.right_stick_y.toDouble(), deadband).powerProfile(power)
    }

    private fun joyStickToAngle(x: Double, y: Double): Rotation2d {
        val theta = atan2(y, x)
        return if (theta >= 0.0) Rotation2d.fromRadians(theta) else Rotation2d.fromRadians(2 * PI + theta)
    }

    /**
     * The angle the right joystick forms with the right side of the X axis.
     * Counter-clockwise positive, goes up to 360 degrees.
     */
    fun getRightAngle(): Rotation2d = joyStickToAngle(getRightX(), getRightY())

    /**
     * The angle the left joystick forms with the right side of the X axis.
     * Counter-clockwise positive, goes up to 360 degrees.
     */
    fun getLeftAngle(): Rotation2d = joyStickToAngle(getLeftX(), getLeftY())
}

fun Double.powerProfile(power: Int): Double {
    return if (power % 2 == 0) {
        this.pow(power) * this.sign
    } else this.pow(power)
}