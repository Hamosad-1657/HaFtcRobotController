package com.hamosad.lib.vision

import java.util.concurrent.TimeUnit
import android.util.Size
import com.hamosad.lib.math.Length
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor


open class HaCamera(
    val hardwareMap: HardwareMap,
    val name: String,
    val visionPortalNumber: Int,
    val processor1: VisionProcessor? = null,
    pixelWidth: Int = 640, pixelLength: Int = 480,
    videoFormat: VisionPortal.StreamFormat = VisionPortal.StreamFormat.YUY2,
    val processor2: VisionProcessor? = null,
    ) {

    private val visionPortalBuilder: VisionPortal.Builder = VisionPortal.Builder()

    init {
        //Define visionPortal builder
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName::class.java, name))
        visionPortalBuilder.setCameraResolution(Size(pixelWidth, pixelLength))
        visionPortalBuilder.setStreamFormat(videoFormat)
        visionPortalBuilder.setLiveViewContainerId(visionPortalNumber)
        visionPortalBuilder.setAutoStopLiveView(true)
        if (processor1 != null) {
            visionPortalBuilder.addProcessor(processor1)
            if (processor2 != null) visionPortalBuilder.addProcessor(processor2)
        }
    }

    val visionPortal: VisionPortal get() = visionPortalBuilder.build()
    val currentFPS: Int get() = visionPortal.fps.toInt()
    val isConnected: Boolean get() = visionPortal.cameraState.name != "ERROR"
    val isStreaming: Boolean get() = visionPortal.cameraState.name == "STREAMING"

    /** Closes the view on the driver station */
    fun closeView() {
        visionPortal.stopLiveView()
    }

    /** Resumes the view on the driver station */
    fun resumeView() {
        visionPortal.resumeLiveView()
    }

    /** The camera will continue taking photos and sending them to processors */
    fun closeCamera() {
        visionPortal.stopStreaming()
    }

    /** The camera will continue taking photos and sending them to processors*/
    fun resumeCamera() {
        visionPortal.resumeStreaming()
    }

    /** The camera shuts down completely, erasing all data and processors, cannot be reversed, must define camera again */
    fun shutDownCamera() {
        visionPortal.close()
    }

    fun enableProcessor(processor: AprilTagProcessor) {
        visionPortal.setProcessorEnabled(processor, true)
    }

    fun disableProcessor(processor: AprilTagProcessor) {
        visionPortal.setProcessorEnabled(processor, false)
    }

    /**
     * Sets the camera's exposure to a specific duration.
     * This is only effective if the camera supports manual exposure control.
     *
     * @param duration The exposure duration.
     * @param unit The time unit for the duration (e.g., TimeUnit.MILLISECONDS).
     * @return True if the exposure was set successfully, false otherwise.
     */
    fun setExposure(duration: Long, unit: TimeUnit): Boolean {
        val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
        if (!exposureControl.isExposureSupported) {
            return false
        }
        exposureControl.mode = ExposureControl.Mode.Manual
        return exposureControl.setExposure(duration, unit)
    }

    /**
     * Sets the camera's gain (ISO).
     * This is only effective if the camera supports gain control.
     *
     * @param gainValue The desired gain value. Must be between the camera's min and max gain.
     * @return True if the gain was set successfully, false otherwise.
     */
    fun setGain(gainValue: Int): Boolean {
        val gainControl = visionPortal.getCameraControl(GainControl::class.java)
        if (gainValue < gainControl.minGain || gainValue > gainControl.maxGain) {
            return false
        }
        gainControl.gain = gainValue
        return true
    }
}