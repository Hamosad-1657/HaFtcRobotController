package com.hamosad.lib.components.motors

import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

enum class Direction {
    FORWARD,
    REVERSE
}

class HaCRServoMotor(name: String, hardwareMap: HardwareMap) {
    private val crServo: CRServo = hardwareMap.get(CRServo::class.java, name)

    fun setVoltage(volts: Volts) {
        crServo.power = volts / 6
    }

    fun stopServo() {
        crServo.power = 0.0
    }

    fun setDirection(direction: Direction) {
        when (direction) {
            Direction.FORWARD -> crServo.direction = DcMotorSimple.Direction.FORWARD
            Direction.REVERSE -> crServo.direction = DcMotorSimple.Direction.REVERSE
        }
    }
}

class HaServoMotor(name: String, hardwareMap: HardwareMap) {
    private val servo: Servo = hardwareMap.get(Servo::class.java, name)
    val currentCommandedPosition get() = servo.position

    fun setPosition(position: Rotation2d) {
        if (0 <= position.asDegrees && position.asDegrees <= 180) {
            servo.position = position.asDegrees
        }
    }
}