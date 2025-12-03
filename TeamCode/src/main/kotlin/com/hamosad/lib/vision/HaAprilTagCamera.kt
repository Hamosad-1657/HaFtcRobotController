package com.hamosad.lib.vision

import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Pose2d
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Translation2d
import com.hamosad.lib.math.Translation3d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.ArrayList
import kotlin.system.measureNanoTime

data class RobotPoseStdDevs(
    val translationX: Double,
    val translationY: Double,
    val rotation: Double
)

data class AprilTagsStdDevs(
    val oneTag: RobotPoseStdDevs,
    val twoTags: RobotPoseStdDevs,
)

class HaAprilTagCamera(
    hardwareMap: HardwareMap,
    name: String,
    visionPortalNumber: Int,
    private val maxTrustRange: Length,
    private val maxDecisionMargin: Double,
    /** X: right, Y Down, Z: forward Meters*/
    cameraPositionMeters: Translation3d,
    cameraOrientation: Rotation3d,
    private val aprilTagStdDevs: AprilTagsStdDevs,
    pixelWidth: Int = 640, pixelLength: Int = 480,
    videoFormat: VisionPortal.StreamFormat = VisionPortal.StreamFormat.YUY2,
    tagFamily: AprilTagProcessor.TagFamily = AprilTagProcessor.TagFamily.TAG_36h11
) :
    HaCamera(hardwareMap, name, visionPortalNumber, run {
        // Define Processor Builder
        val aprilTagProcessorBuilder: AprilTagProcessor.Builder = AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setCameraPose(
                Position(
                    DistanceUnit.METER,
                    cameraPositionMeters.x,
                    cameraPositionMeters.y,
                    cameraPositionMeters.z,
                    measureNanoTime {}
                ),
                YawPitchRollAngles(
                    AngleUnit.DEGREES,
                    cameraOrientation.yawAngle.asDegrees,
                    cameraOrientation.pitchAngle.asDegrees,
                    cameraOrientation.rollAngle.asDegrees,
                    measureNanoTime {}
                )
            )
            .setTagFamily(tagFamily)
        aprilTagProcessor = aprilTagProcessorBuilder.build()
        val calibrationIdentity = CameraCalibrationIdentity({ true })

        val cameraCalibration: CameraCalibration = CameraCalibration(
            calibrationIdentity,
            intArrayOf(pixelWidth, pixelLength),
            floatArrayOf(668.038F, 668.038F),
            floatArrayOf(310.853F, 229.224F),
            floatArrayOf(   0.16474F, -1.32689F, 2.49793F,  0.000256529F, 0.00167112F, 0F, 0F, 0F),
            false,
            false
        )

        aprilTagProcessor.init(pixelWidth, pixelLength, cameraCalibration)
        aprilTagProcessor
    }, pixelWidth, pixelLength, videoFormat) {
    companion object {
        private lateinit var aprilTagProcessor: AprilTagProcessor
    }

    val hasTargets: Boolean get() = aprilTagProcessor.detections.isNotEmpty()
    val allTargets: ArrayList<AprilTagDetection?>? get() = aprilTagProcessor.detections
    val closestTarget: AprilTagDetection?
        get() = if (hasTargets) {
            allTargets!!.minByOrNull { it!!.ftcPose.range }
        } else {
            null
        }
    val isInRange: Boolean
        get() {
            if (hasTargets) {
                return closestTarget!!.ftcPose.range < maxTrustRange.asInches
            }
            return false
        }

    val poseEstimationStdDevs
        get() =
            if (allTargets?.size == 1) {
                aprilTagStdDevs.oneTag
            } else {
                aprilTagStdDevs.twoTags
            }

    // the estimated pose is given in values of double I don't know why
    val estimatedPose: Pose2d?
        get() {
            if (!hasTargets || !isInRange || closestTarget == null || allTargets == null) return null
            if (maxDecisionMargin < closestTarget!!.decisionMargin) return null

            val bestPose = Pose2d(
                Translation2d(
                    closestTarget!!.robotPose.position.x,
                    closestTarget!!.robotPose.position.y
                ),
                Rotation2d.fromRadians(closestTarget!!.robotPose.orientation.getYaw(AngleUnit.RADIANS)),
                RobotPoseStdDevs(0.0, 0.0, 0.0)
            )

            for (i in allTargets!!) {
                bestPose.addPoseEstimate(
                    Pose2d(
                        Translation2d(i!!.robotPose.position.x, i.robotPose.position.y),
                        Rotation2d.fromRadians(i.robotPose.orientation.getYaw(AngleUnit.RADIANS)),
                        //Had to use decision margin, no other info about quality of detections. Im so sorry
                        RobotPoseStdDevs(
                            i.decisionMargin.toDouble(),
                            i.decisionMargin.toDouble(),
                            i.decisionMargin.toDouble()
                        )
                    )
                )
            }

            return bestPose
        }

    fun isTagDetected(tagId: Int): Boolean {
        if (hasTargets) {
            for (i in allTargets!!) {
                if (i!!.id == tagId) {
                    return true
                }
            }
        }
        return false
    }
}
