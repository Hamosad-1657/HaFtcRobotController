package com.hamosad.lib.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class HaCamera(
    val hardwareMap: HardwareMap,
    val cameraPosition: Position,
    val cameraOrientation: YawPitchRollAngles,
    val tagFamily: AprilTagProcessor.TagFamily,
    val name: String) {
    private val aprilTagProcessorBuilder: AprilTagProcessor.Builder = AprilTagProcessor.Builder()

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
        val visionPortalBuilder: VisionPortal.Builder = VisionPortal.Builder()
    }
}