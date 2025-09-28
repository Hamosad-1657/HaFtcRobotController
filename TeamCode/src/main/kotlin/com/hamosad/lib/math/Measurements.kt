package com.hamosad.lib.math

private const val INCH_TO_METER_RATIO = 0.0254

class Length private constructor(private var lengthMeters: Double) {
    companion object {
        fun fromMeters(lengthMeters: Double): Length = Length(lengthMeters)

        fun fromInches(lengthInches: Double): Length = Length(lengthInches * INCH_TO_METER_RATIO)

        fun fromCentimeters(lengthCentimeters: Double): Length = Length(lengthCentimeters / 100)

        fun fromMillimeters(lengthMillimeters: Double): Length = Length(lengthMillimeters / 1000)
    }

    fun asMeters() = lengthMeters
    fun asInches() = lengthMeters / INCH_TO_METER_RATIO
    fun asCentimeters() = lengthMeters * 100
    fun asMillimeters() = lengthMeters * 1000
}