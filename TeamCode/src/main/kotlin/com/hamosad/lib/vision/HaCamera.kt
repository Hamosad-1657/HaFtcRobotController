package com.hamosad.lib.vision

import android.util.Size
import com.hamosad.lib.math.Length
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor


class HaCamera(
    val hardwareMap: HardwareMap,
    val cameraPosition: Position,
    val cameraOrientation: YawPitchRollAngles,
    val tagFamily: AprilTagProcessor.TagFamily,
    val name: String,
    visionRange: Length,
    pixelWidth: Int, pixelLength: Int,
    videoFormat: VisionPortal.StreamFormat = VisionPortal.StreamFormat.YUY2) {
    private val aprilTagProcessorBuilder: AprilTagProcessor.Builder = AprilTagProcessor.Builder()
    private val visionPortalBuilder: VisionPortal.Builder = VisionPortal.Builder()

    init {
        // Define Processor Builder
        aprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
        aprilTagProcessorBuilder.setDrawTagID(true)
        aprilTagProcessorBuilder.setDrawTagOutline(true)
        aprilTagProcessorBuilder.setDrawAxes(true)
        aprilTagProcessorBuilder.setDrawCubeProjection(true)
        aprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation)
        aprilTagProcessorBuilder.setTagFamily(tagFamily)

        //Define visionPortal builder
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName::class.java, name))
        visionPortalBuilder.addProcessor(aprilTagProcessorBuilder.build())
        visionPortalBuilder.setCameraResolution(Size(pixelWidth, pixelLength))
        visionPortalBuilder.setStreamFormat(videoFormat)
        visionPortalBuilder.enableLiveView(true)
        visionPortalBuilder.setAutoStopLiveView(true)
    }

    val detections = aprilTagProcessorBuilder.build().detections
    val hasTargets: Boolean get() = detections.isNotEmpty()
}