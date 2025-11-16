package com.hamosad.lib.components.sensors

import com.hamosad.lib.math.Rotation2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

class HaIMU(hardwareMap: HardwareMap, name: String) {
    private val imu: IMU = hardwareMap.get(IMU::class.java, name)

    init {
        imu.resetYaw()
    }

    val currentYaw: Rotation2d get() =
        Rotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle.toDouble()
        )
    val currentPitch: Rotation2d get() =
        Rotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle.toDouble()
        )
    val currentRoll: Rotation2d get() =
        Rotation2d.fromDegrees(
            imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle.toDouble()
        )

    fun resetYaw() {
        imu.resetYaw()
    }
}