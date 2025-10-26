package com.hamosad.lib.components.motors

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Volts
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients

enum class DCMotorStopMode {
    COAST,
    BRAKE,
}


class HaMotor(name: String, hardwareMap: HardwareMap, val ticksPerRotation: Int) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, name)
    val currentVelocity get() = motor.velocity
    val currentPosition get() = motor.currentPosition

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun stopMotor() {
        motor.power = 0.0
    }

    fun setDirection(direction: DcMotorSimple.Direction) {
        motor.direction = direction
    }

    fun resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    }

    fun setVoltage(voltage: Volts) {
        motor.power = voltage / 12
    }

    fun setPosition(position: Rotation2d) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        motor.setTargetPosition(position.asRotations.toInt() * ticksPerRotation)
    }

    fun setVelocity(velocity: AngularVelocity) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        (motor as DcMotorEx).setVelocity(velocity.asRPM * ticksPerRotation * 60)
    }

    fun setStopMode(dcMotorStopMode: DCMotorStopMode) {
        when (dcMotorStopMode) {
            DCMotorStopMode.COAST -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            DCMotorStopMode.BRAKE -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    fun setPid(p: Double = 0.0, i: Double = 0.0, d: Double = 0.0, f: Double = 0.0) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients(p, i, d, f))
    }
}