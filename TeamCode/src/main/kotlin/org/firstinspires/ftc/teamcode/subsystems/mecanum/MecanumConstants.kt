package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.MPS
import com.hamosad.lib.math.PIDGains
import kotlin.math.PI

object MecanumConstants {
    val MAX_WHEEL_SPEED: AngularVelocity = AngularVelocity.fromRPM(312.0)
    val WHEEL_RADIUS: Length = Length.fromMillimeters(52.0)
    val CHASSIS_RADIUS: Length = Length.fromMillimeters(52.0)

    val MAX_CHASSIS_SPEED: MPS = MAX_WHEEL_SPEED.asRPS * WHEEL_RADIUS.asMeters * 2 * PI

    val MAX_CHASSIS_ANGULAR_VELOCITY: AngularVelocity = AngularVelocity.fromRPS(6.0)

    val wheelGains: PIDGains = PIDGains(
        p = 5.0,
        i = 2.0,
        d = 0.0,
    )
}