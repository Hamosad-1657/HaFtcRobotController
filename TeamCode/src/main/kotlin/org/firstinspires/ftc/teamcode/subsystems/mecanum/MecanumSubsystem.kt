package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.components.motors.HaMotor
import com.hamosad.lib.components.motors.MotorType
import com.hamosad.lib.components.sensors.HaIMU
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.PIDController
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Translation2d
import com.hamosad.lib.math.Translation3d
import com.hamosad.lib.vision.AprilTagsStdDevs
import com.hamosad.lib.vision.HaAprilTagCamera
import com.hamosad.lib.vision.HaCamera
import com.hamosad.lib.vision.RobotPoseStdDevs
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumKinematics as Kinematics

object MecanumSubsystem: Subsystem() {
    // FL, BR, FR, BL
    private var motors: List<HaMotor> = listOf()
    private val controllers: List<PIDController> = listOf(
        PIDController(Constants.wheelGains),
        PIDController(Constants.wheelGains),
        PIDController(Constants.wheelGains),
        PIDController(Constants.wheelGains),
    )
    private var imu: HaIMU? = null
    var camera: HaCamera? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        motors = listOf(
            HaMotor("FL", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("FR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BL", hardwareMap!!, MotorType.GO_BUILDA5202),
        )
        imu = HaIMU(hardwareMap!!, "IMU")

        camera = HaCamera(hardwareMap!!, "camera", 0)
        camera?.resumeView()
    }

    private val currentAngle: Rotation2d
        get() = imu?.currentYaw ?: Rotation2d.fromDegrees(0.0)

    // Low level functions
    fun resetGyro() {
        imu?.resetYaw()
    }

    // Low level motor PID
    private var wheelVelocitySetpoints: List<AngularVelocity> = listOf()

    /** Called in loops to control PID of motors. */
    private fun controlMotors(newSetpoints: List<AngularVelocity> = wheelVelocitySetpoints) {
        wheelVelocitySetpoints = newSetpoints

        for (i in 0..3) {
            motors[i].setVoltage(controllers[i].calculate(motors[i].currentVelocity.asRPS, wheelVelocitySetpoints[i].asRPS))
        }
    }

    fun spinClockwise(angularVelocity: AngularVelocity) {
        controlMotors(Kinematics.angularVelocityToMotorVelocities(angularVelocity))
    }

    fun drive(fieldRelative: Boolean, chassisSpeeds: ChassisSpeeds) {
        val updatedSpeeds = if (fieldRelative) ChassisSpeeds(
            Translation2d(
                chassisSpeeds.translation.length,
                chassisSpeeds.translation.rotation - currentAngle
            ),
            chassisSpeeds.omega
        ) else chassisSpeeds

        controlMotors(Kinematics.chassisSpeedsToMotorVelocities(updatedSpeeds))
    }



    // Periodic
    override fun periodic() {

    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry) {
        telemetry.addData("Angle deg", currentAngle.asDegrees)
        telemetry.addData("is the camera streaming", camera?.isStreaming)
        telemetry.addData("is the camera connected", camera?.isConnected)
    }
}