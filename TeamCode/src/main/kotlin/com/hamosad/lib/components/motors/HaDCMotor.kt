package com.hamosad.lib.components.motors

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.PIDController
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

enum class MotorType(val ticksPerRotation: Double) {
    GO_BUILDA5202(537.7),
}


class HaMotor(name: String, hardwareMap: HardwareMap, val type: MotorType) {
    private val motor = hardwareMap.get(DcMotorEx::class.java, name)
    private val controller = PIDController(0.0, 0.0, 0.0)

    val currentVelocity get() = AngularVelocity.fromRPS(motor.velocity / type.ticksPerRotation)
    val currentPosition get() = motor.currentPosition

    init {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun configPID(p: Double = 0.0, i: Double = 0.0, d: Double = 0.0) {
        controller.updateGains(p, i, d)
    }

    fun setFeedForward(feedforward: Double) {
        controller.updateGains(newFeedForward = feedforward)
    }

    fun setDirection(direction: DcMotorSimple.Direction) {
        motor.direction = direction
    }

    fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun setVoltage(voltage: Volts) {
        motor.power = voltage / 12
    }

    fun stopMotor() {
        motor.power = 0.0
    }

    fun setPosition(position: Rotation2d) {

    }

    fun setVelocity(velocity: AngularVelocity) {

    }

    fun setStopMode(dcMotorStopMode: DCMotorStopMode) {
        when (dcMotorStopMode) {
            DCMotorStopMode.COAST -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            DCMotorStopMode.BRAKE -> motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}