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
import com.hamosad.lib.vision.RobotPoseStdDevs
import com.qualcomm.robotcore.hardware.DcMotorSimple
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

    var USE_VISION = false
    var aprilTagCamera: HaAprilTagCamera? = null

    override fun init(newHardwareMap: HardwareMap) {
        super.init(newHardwareMap)
        motors = listOf(
            HaMotor("FL", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("FR", hardwareMap!!, MotorType.GO_BUILDA5202),
            HaMotor("BL", hardwareMap!!, MotorType.GO_BUILDA5202),
        )
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE)
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD)
        motors[2].setDirection(DcMotorSimple.Direction.FORWARD)
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE)

        imu = HaIMU(hardwareMap!!, "IMU")

        if (USE_VISION) {
            aprilTagCamera = HaAprilTagCamera(
                hardwareMap!!,
                "camera",
                Length.fromMeters(5.0),
                7.0,
                Translation3d(0.0, 0.0, 0.18),
                Rotation3d(
                    Rotation2d.fromDegrees(45.0),
                    Rotation2d.fromDegrees(0.0),
                    Rotation2d.fromDegrees(0.0)
                ),
                AprilTagsStdDevs(
                    RobotPoseStdDevs(0.0, 0.0, 0.0),
                    RobotPoseStdDevs(0.0 ,0.0 ,0.0)
                )
            )
        }
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
    // TODO: DOESN'T WORK STUPID SHIT RAHHHHHHHHHHHHHHHHHH
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

    // Testing
    fun testMotors(motor: Int) {
        motors[motor].setVoltage(4.0)
    }

    fun testPID() {
        for (i in 0..3) {
            motors[i].setVoltage(controllers[i].calculate(motors[i].currentVelocity.asRPM, 2.0))
        }
    }

    // Periodic
    override fun periodic() {

    }

    // Telemetry
    override fun updateTelemetry(telemetry: Telemetry) {
        telemetry.addData("Angle deg", currentAngle.asDegrees)

        // FL, BR, FR, BL
        if(wheelVelocitySetpoints.lastIndex == 3) {
        telemetry.addData("Commanded velocity FL", wheelVelocitySetpoints[0].asRPM)
        telemetry.addData("Commanded velocity BR", wheelVelocitySetpoints[1].asRPM)
        telemetry.addData("Commanded velocity FR", wheelVelocitySetpoints[2].asRPM)
        telemetry.addData("Commanded velocity BL", wheelVelocitySetpoints[3].asRPM)
        } else {
            telemetry.addData("Commanded velocity FL", 0.0)
            telemetry.addData("Commanded velocity BR", 0.0)
            telemetry.addData("Commanded velocity FR", 0.0)
            telemetry.addData("Commanded velocity BL", 0.0)
        }
        // telemetry.addData("Pose X", aprilTagCamera!!.estimatedPose?.translation2d?.x)
        // telemetry.addData("Pose Y", aprilTagCamera!!.estimatedPose?.translation2d?.y)
        // telemetry.addData("Pose Rotation", aprilTagCamera!!.estimatedPose?.rotation2d?.asDegrees)

    }
}