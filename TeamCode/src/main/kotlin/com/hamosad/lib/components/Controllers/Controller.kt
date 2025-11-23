package com.hamosad.lib.components.Controllers

import com.hamosad.lib.commands.Trigger
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.continuousDeadband
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

/**
 * Automatically flips y value of joysticks and applies deadband
 */
class HaCommandController(val controller: () -> Gamepad?, val deadband: Double, private val power: Int = 1) {
    fun triangle(): Trigger = Trigger { controller()?.triangle ?: false }
    fun circle(): Trigger = Trigger { controller()?.circle ?: false }
    fun cross(): Trigger = Trigger { controller()?.cross ?: false }
    fun square(): Trigger = Trigger { controller()?.square ?: false }

    fun rJoyPressed(): Trigger = Trigger { controller()?.right_stick_button ?: false }
    fun lJoyPressed(): Trigger = Trigger { controller()?.left_stick_button ?: false }

    fun options(): Trigger = Trigger { controller()?.options ?: false }
    fun share(): Trigger = Trigger { controller()?.share ?: false }
    fun psButton(): Trigger = Trigger { controller()?.ps ?: false }

    fun dpadUp(): Trigger = Trigger { controller()?.dpad_up ?: false }
    fun dpadRight(): Trigger = Trigger { controller()?.dpad_right ?: false }
    fun dpadDown(): Trigger = Trigger { controller()?.dpad_down ?: false }
    fun dpadLeft(): Trigger = Trigger { controller()?.dpad_left ?: false }

    fun r1(): Trigger = Trigger { controller()?.right_bumper ?: false }
    fun l1(): Trigger = Trigger { controller()?.left_bumper ?: false }
    fun r2Pressed(): Trigger = Trigger { (controller()?.right_trigger ?: 0.0f) > 0.7 }
    fun l2Pressed(): Trigger = Trigger { (controller()?.left_trigger ?: 0.0f) > 0.7 }

    fun getR2(): Double = controller()?.right_trigger?.toDouble() ?: 0.0
    fun getL2(): Double = controller()?.left_trigger?.toDouble() ?: 0.0

    fun getLeftX(): Double {
        return continuousDeadband(controller()?.left_stick_x?.toDouble() ?: 0.0, deadband).powerProfile(power)
    }

    fun getLeftY(): Double {
        return -continuousDeadband(controller()?.left_stick_y?.toDouble() ?: 0.0, deadband).powerProfile(power)
    }

    fun getRightX(): Double {
        return continuousDeadband(controller()?.right_stick_x?.toDouble() ?: 0.0, deadband).powerProfile(power)
    }

    fun getRightY(): Double {
        return -continuousDeadband(controller()?.right_stick_y?.toDouble() ?: 0.0, deadband).powerProfile(power)
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