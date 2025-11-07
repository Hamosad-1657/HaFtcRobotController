package com.hamosad.lib.vision

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class HaCamera(val aprilTagLibrary: AprilTagLibrary) {
    private val aprilTagProcessorBuilder: AprilTagProcessor.Builder = AprilTagProcessor.Builder()

    // Optional: specify a custom Library of AprilTags.
    init {
        aprilTagProcessorBuilder.setTagLibrary(aprilTagLibrary)
        // Optional: set other custom features of the AprilTag Processor (4 are shown here).
        aprilTagProcessorBuilder.setDrawTagID(true);       // Default: true, for all detections.
        aprilTagProcessorBuilder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        aprilTagProcessorBuilder.setDrawAxes(true);        // Default: false.
        aprilTagProcessorBuilder.setDrawCubeProjection(true);        // Default: false.
    }   // The OpMode must have already created a Library.

    // Create an AprilTagProcessor by calling build()
    public val aprilTagProcessor = aprilTagProcessorBuilder.build();
}