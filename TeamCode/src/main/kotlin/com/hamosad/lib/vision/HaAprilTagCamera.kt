package com.hamosad.lib.vision

import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Pose2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Translation2d
import com.hamosad.lib.math.Translation3d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
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
    pixelWidth: Int = 640, pixelLength: Int = 480,
    videoFormat: VisionPortal.StreamFormat = VisionPortal.StreamFormat.YUY2,
    private val maxTrustRange: Length,
    private val maxDecisionMargin: Double,
    cameraPositionMeters: Translation3d,
    cameraOrientation: Rotation3d,
    private val aprilTagStdDevs: AprilTagsStdDevs,
    tagFamily: AprilTagProcessor.TagFamily = AprilTagProcessor.TagFamily.TAG_36h11
):
    HaCamera(hardwareMap, name, aprilTagProcessor, pixelWidth, pixelLength, videoFormat) {
    companion object {
        private lateinit var aprilTagProcessor: AprilTagProcessor
    }

    init {
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
                    measureNanoTime{}
                ),
                YawPitchRollAngles(
                    AngleUnit.DEGREES,
                    cameraOrientation.yawAngleRotations * 360,
                    cameraOrientation.pitchAngleRotations * 360,
                    cameraOrientation.rollAngleRotations * 360,
                    measureNanoTime {}
                ))
            .setTagFamily(tagFamily)
        aprilTagProcessor = aprilTagProcessorBuilder.build()
    }

    val hasTargets: Boolean get() = aprilTagProcessor.detections.isNotEmpty()
    val allTargets: ArrayList<AprilTagDetection?>? get() = aprilTagProcessor.detections
    val closestTarget: AprilTagDetection? get() = if (hasTargets) {allTargets!!.minByOrNull { it!!.ftcPose.range }} else {null}
    val isInRange: Boolean get() {
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
    val estimatedPose: Pose2d? get() {
        if (!hasTargets || !isInRange || closestTarget == null || allTargets == null) null
        if (maxDecisionMargin < closestTarget!!.decisionMargin) null

        val bestPose = Pose2d(
            Translation2d(closestTarget!!.robotPose.position.x, closestTarget!!.robotPose.position.y),
            Rotation2d.fromRadians(closestTarget!!.robotPose.orientation.getYaw(AngleUnit.RADIANS)),
            RobotPoseStdDevs(0.0, 0.0, 0.0)
        )

        for (i in allTargets!!) {
            bestPose.addPoseEstimate(
                Pose2d(
                    Translation2d(i!!.robotPose.position.x, i.robotPose.position.y),
                    Rotation2d.fromRadians(i.robotPose.orientation.getYaw(AngleUnit.RADIANS)),
                    //Had to use decision margin, no other info about quality of detections. Im so sorry
                    RobotPoseStdDevs(i.decisionMargin.toDouble(), i.decisionMargin.toDouble(), i.decisionMargin.toDouble())
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