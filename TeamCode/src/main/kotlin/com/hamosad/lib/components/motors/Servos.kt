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
    val cRServo: CRServo = hardwareMap.get(CRServo::class.java, name)
    val currentPosition get() = cRServo.controller.getServoPosition(cRServo.portNumber)

    fun setVoltage(volts: Volts) {
        cRServo.power = volts / 6
    }

    fun setDirection(direction: Direction) {
        when (direction) {
            Direction.FORWARD -> cRServo.direction = DcMotorSimple.Direction.FORWARD
            Direction.REVERSE -> cRServo.direction = DcMotorSimple.Direction.REVERSE
        }
    }

    fun setTargetAngle(angle: Rotation2d) {
        cRServo.controller.setServoPosition(cRServo.portNumber, angle.asRotations)
    }
}

class HaServoMotor(name: String, hardwareMap: HardwareMap) {
    val servo: Servo = hardwareMap.get(Servo::class.java, name)
    val currentPosition get() = servo.position

    fun setPosition(position: Rotation2d) {
        if (0 < position.asDegrees && position.asDegrees < 180) {
            servo.position = position.asDegrees
        }
    }
}