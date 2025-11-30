package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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
import com.hamosad.lib.vision.HaCamera
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

    var USE_VISION = true
    var haCamera: HaCamera? = null

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
            haCamera = HaCamera(
                hardwareMap!!,
                "Webcam 1",
                0,
            )
            haCamera?.resumeView()
            haCamera?.resumeCamera()
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
    private fun controlMotors(newSetpoints: List<AngularVelocity> = wheelVelocitySetpoints) {
        wheelVelocitySetpoints = newSetpoints

        for (i in 0..3) {
            motors[i].setVoltage(controllers[i].calculate(motors[i].currentVelocity.asRPS, wheelVelocitySetpoints[i].asRPS))
        }
    }

    fun spinClockwise(angularVelocity: AngularVelocity) {
        controlMotors(Kinematics.angularVelocityToMotorVelocities(angularVelocity))
    }

    var requestedChassisSpeedsTranslation: Translation2d = Translation2d(0.0, 0.0)
    fun drive(fieldRelative: Boolean, chassisSpeeds: ChassisSpeeds) {
        val updatedSpeeds = if (fieldRelative) ChassisSpeeds(
            Translation2d(
                chassisSpeeds.translation.length,
                chassisSpeeds.translation.rotation - currentAngle
            ),
            chassisSpeeds.omega
        ) else chassisSpeeds

        requestedChassisSpeedsTranslation = chassisSpeeds.translation
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
    override fun updateTelemetry(telemetry: Telemetry, dashboardPacket: TelemetryPacket) {

        telemetry.addData("Angle deg", currentAngle.asDegrees)
        telemetry.addData("Requested chassis speeds angle deg", requestedChassisSpeedsTranslation.rotation.asDegrees)

//        // FL, BR, FR, BL
//        if(wheelVelocitySetpoints.lastIndex == 3) {
//        telemetry.addData("Commanded velocity FL RPM", wheelVelocitySetpoints[0].asRPM)
//        telemetry.addData("Commanded velocity BR RPM", wheelVelocitySetpoints[1].asRPM)
//        telemetry.addData("Commanded velocity FR RPM", wheelVelocitySetpoints[2].asRPM)
//        telemetry.addData("Commanded velocity BL RPM", wheelVelocitySetpoints[3].asRPM)
//        } else {
//            telemetry.addData("Commanded velocity FL RPM", 0.0)
//            telemetry.addData("Commanded velocity BR RPM", 0.0)
//            telemetry.addData("Commanded velocity FR RPM", 0.0)
//            telemetry.addData("Commanded velocity BL RPM", 0.0)
//        }
         telemetry.addData("Pis camera streaming", haCamera?.isConnected)
//
//        if (wheelVelocitySetpoints.size == 4) {
//            dashboardPacket.put("FL setpoint RPM", wheelVelocitySetpoints[0].asRPM)
//            dashboardPacket.put("BR setpoint RPM", wheelVelocitySetpoints[1].asRPM)
//            dashboardPacket.put("FR setpoint RPM", wheelVelocitySetpoints[2].asRPM)
//            dashboardPacket.put("BL setpoint RPM", wheelVelocitySetpoints[3].asRPM)
//
//            dashboardPacket.put("FL velocity RPM", motors[0].currentVelocity.asRPM)
//            dashboardPacket.put("BR velocity RPM", motors[1].currentVelocity.asRPM)
//            dashboardPacket.put("FR velocity RPM", motors[2].currentVelocity.asRPM)
//            dashboardPacket.put("BL velocity RPM", motors[3].currentVelocity.asRPM)
//        }
    }
}