package com.hamosad.lib.components.motors

import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class HaCRServoMotor(name: String, hardwareMap: HardwareMap) {
    private val crServo: CRServo = hardwareMap.get(CRServo::class.java, name)

    var direction: DcMotorSimple.Direction
        get() = crServo.direction
        set(value) {
            crServo.direction = value
        }

    fun setVoltage(volts: Volts) {
        crServo.power = volts / 6
    }

    fun stopServo() {
        crServo.power = 0.0
    }
}

class HaServoMotor(name: String, hardwareMap: HardwareMap, private val angleRange: Rotation2d = Rotation2d.fromDegrees(180.0)) {
    private val servo: Servo = hardwareMap.get(Servo::class.java, name)
    var currentCommandedPosition: Rotation2d
        get() = Rotation2d.fromRotations(servo.position * angleRange.asRotations)
        set(value) {
            if (value.asDegrees in 0.0..angleRange.asDegrees) {
                servo.position = value.asRotations / angleRange.asRotations
            }
        }
    var direction: Servo.Direction
        get() = servo.direction
        set(value) {
            servo.direction = value
        }
}