package com.hamosad.lib.vision

import androidx.annotation.ColorInt
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.SortOrder
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion

class HaColorCamera(
    hardwareMap: HardwareMap,
    name: String,
    colorRange: ColorRange,
    erodeSize: Int,
    dilateSize: Int,
    blurSize: Int,
    blobCriteria: ColorBlobLocatorProcessor.BlobCriteria, // I fucking hate blobs P.S hate them so much, Amit
    minBlobFilter: Double, maxBlobFilter: Double,
    roiRegion: ImageRegion?,
    contourColor: Int?,
    boxFitColor: Int?,
    pixelWidth: Int = 640, pixelLength: Int = 480,
) : HaCamera(hardwareMap, name, colorProcessor, pixelWidth, pixelLength) {

    companion object {
        private lateinit var colorProcessor: ColorBlobLocatorProcessor
    }

    init {
        //Setup the colorProcessor builder
        val colorProcessorBuilder: ColorBlobLocatorProcessor.Builder = ColorBlobLocatorProcessor.Builder()
        colorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        colorProcessorBuilder.setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.OPENING)
        colorProcessorBuilder.setErodeSize(erodeSize)
        colorProcessorBuilder.setDilateSize(dilateSize)
        colorProcessorBuilder.setDrawContours(true)
        colorProcessorBuilder.setBlurSize(blurSize)
        colorProcessorBuilder.setTargetColorRange(colorRange)
        if (roiRegion != null) colorProcessorBuilder.setRoi(roiRegion)

        //Set colors for things
        if (boxFitColor != null) colorProcessorBuilder.setBoxFitColor(boxFitColor)
        if (contourColor != null) colorProcessorBuilder.setContourColor(contourColor)

        colorProcessor = colorProcessorBuilder.build()
        colorProcessor.setSort(ColorBlobLocatorProcessor.BlobSort(blobCriteria, SortOrder.DESCENDING))
        colorProcessor.addFilter(ColorBlobLocatorProcessor.BlobFilter(blobCriteria, minBlobFilter, maxBlobFilter))
    }
    
    val results get() = colorProcessor.blobs

}