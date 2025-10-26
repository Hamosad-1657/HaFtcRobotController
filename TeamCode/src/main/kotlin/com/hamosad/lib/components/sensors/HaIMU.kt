package com.hamosad.lib.components.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware

class HaIMU(hardwareMap: HardwareMap) {
    private val imu: IMU = hardwareMap.get(IMU::class.java, "IMU")

    //val currentYaw: Double get() =  getRobotYawPitchRollAngles().getYaw(BNO055IMU.AngleUnit.DEGREES);
}