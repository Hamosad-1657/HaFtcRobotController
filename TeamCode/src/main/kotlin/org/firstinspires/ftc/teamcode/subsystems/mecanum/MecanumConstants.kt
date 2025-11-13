package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.PIDGains

object MecanumConstants {
    val MAX_ROTATION_SPEED: AngularVelocity = AngularVelocity.fromRPM(312.0)
    val WHEEL_CIRCUMFERENCE: Length = Length.fromMillimeters(52.0)
    val CHASSIS_CIRCUMFERENCE: Length = Length.fromMillimeters(52.0)

    val wheelGains: PIDGains = PIDGains(
        p = 0.0,
        i = 0.0,
        d = 0.0,
    )
}