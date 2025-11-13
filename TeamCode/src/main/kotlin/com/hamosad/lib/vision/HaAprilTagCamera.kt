package com.hamosad.lib.vision

import android.graphics.drawable.GradientDrawable
import com.hamosad.lib.math.Length
import com.hamosad.lib.math.Rotation2d
import com.hamosad.lib.math.Rotation3d
import com.hamosad.lib.math.Translation3d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.system.measureNanoTime

data class stdDevs(
    val translationX: Double,
    val translationY: Double,
    val rotation: Double,
)

class HaAprilTagCamera(
    hardwareMap: HardwareMap,
    name: String,
    val maxTrustRange: Length,
    cameraPositionMeters: Translation3d,
    cameraOrientation: Rotation3d,
    val stdDevs: stdDevs,
    tagFamily: AprilTagProcessor.TagFamily = AprilTagProcessor.TagFamily.TAG_36h11
):
    HaCamera(hardwareMap, name, aprilTagProcessor) {
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
    val allTargets get() = aprilTagProcessor.detections
    val closestTarget get() = allTargets.minByOrNull { it.ftcPose.range }
    val isInRange: Boolean get() {
        if (hasTargets) {
            return closestTarget!!.ftcPose.range < maxTrustRange.asInches
        }
        return false
    }

    fun isTagDetected(tagId: Int): Boolean {
        if (hasTargets == true) {
            for (i in allTargets!!) {
                if (i.id == tagId) {
                    return true
                }
            }
        }
        return false
    }
}