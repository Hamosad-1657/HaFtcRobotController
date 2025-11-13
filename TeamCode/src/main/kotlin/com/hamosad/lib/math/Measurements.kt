package com.hamosad.lib.math

import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tanh

private const val INCH_TO_METER_RATIO = 0.0254

class Length private constructor(private val lengthMeters: Double) {
    companion object {
        fun fromMeters(lengthMeters: Double): Length = Length(lengthMeters)

        fun fromInches(lengthInches: Double): Length = Length(lengthInches * INCH_TO_METER_RATIO)

        fun fromCentimeters(lengthCentimeters: Double): Length = Length(lengthCentimeters / 100)

        fun fromMillimeters(lengthMillimeters: Double): Length = Length(lengthMillimeters / 1000)
    }

    val asMeters get() = lengthMeters
    val asInches get () = lengthMeters / INCH_TO_METER_RATIO
    val asCentimeters get() = lengthMeters * 100
    val asMillimeters get() = lengthMeters * 1000

    operator fun plus(other: Length): Length = Length(this.lengthMeters + other.lengthMeters)
    operator fun minus(other: Length): Length = Length(this.lengthMeters - other.lengthMeters)
    operator fun times(other: Double): Length = Length(this.lengthMeters * other)
    operator fun times(other: Int): Length = Length(this.lengthMeters * other)
    operator fun div(other: Double): Length = Length(this.lengthMeters / other)
    operator fun div(other: Int): Length = Length(this.lengthMeters / other)
}

class Rotation2d private constructor(private val angleRotations: Double) {
    companion object {
        fun fromRotations(angleRotations: Double): Rotation2d = Rotation2d(angleRotations)

        fun fromDegrees(angleDegrees: Double): Rotation2d = Rotation2d(angleDegrees / 360)

        fun fromRadians(angleRadians: Double): Rotation2d = Rotation2d(angleRadians / (2 * PI))
    }

    val asRotations get() = angleRotations
    val asDegrees get() = angleRotations * 360
    val asRadians get() = angleRotations * 2 * PI

    operator fun plus(other: Rotation2d): Rotation2d = Rotation2d(this.angleRotations + other.angleRotations)
    operator fun minus(other: Rotation2d): Rotation2d = Rotation2d(this.angleRotations - other.angleRotations)
    operator fun times(other: Double): Rotation2d = Rotation2d(this.angleRotations * other)
    operator fun times(other: Int): Rotation2d = Rotation2d(this.angleRotations * other)
    operator fun div(other: Double): Rotation2d = Rotation2d(this.angleRotations / other)
    operator fun div(other: Int): Rotation2d = Rotation2d(this.angleRotations / other)
}

class Rotation3d private constructor(
    val pitchAngleRotations: Double,
    val yawAngleRotations: Double,
    val rollAngleRotations: Double
    ) {
    companion object {
        fun fromRotations(pitchAngleRotations: Double, yawAngleRotations: Double, rollAngleRotations: Double): Rotation3d =
            Rotation3d(pitchAngleRotations, yawAngleRotations, rollAngleRotations)

        fun fromDegrees(pitchAngleRotations: Double, yawAngleRotations: Double, rollAngleRotations: Double): Rotation3d =
            Rotation3d(pitchAngleRotations / 360, yawAngleRotations / 360, rollAngleRotations / 360)

        fun fromRadians(pitchAngleRotations: Double, yawAngleRotations: Double, rollAngleRotations: Double): Rotation3d =
            Rotation3d(pitchAngleRotations / (2 * PI), yawAngleRotations / (2 * PI), rollAngleRotations / (2 * PI))
    }

    val asRotations get() = Triple(pitchAngleRotations, yawAngleRotations, rollAngleRotations)
    val asDegrees get() = Triple(pitchAngleRotations * 360, yawAngleRotations * 360, rollAngleRotations * 360)
    val asRadians get() = Triple(pitchAngleRotations * 2 * PI, yawAngleRotations * 2 * PI, rollAngleRotations * 2* PI)


    operator fun plus(other: Rotation3d): Rotation3d = Rotation3d(
        this.pitchAngleRotations + other.pitchAngleRotations,
        this.yawAngleRotations + other.yawAngleRotations,
        this.rollAngleRotations + other.rollAngleRotations
    )
    operator fun minus(other: Rotation3d): Rotation3d = Rotation3d(
        this.pitchAngleRotations - other.pitchAngleRotations,
        this.yawAngleRotations - other.yawAngleRotations,
        this.rollAngleRotations - other.rollAngleRotations
    )
    operator fun times(other: Double): Rotation3d = Rotation3d(
        this.pitchAngleRotations * other,
        this.yawAngleRotations * other,
        this.rollAngleRotations * other
    )
    operator fun times(other: Int): Rotation3d = Rotation3d(
        this.pitchAngleRotations * other,
        this.yawAngleRotations * other,
        this.rollAngleRotations * other
    )
    operator fun div(other: Double): Rotation3d = Rotation3d(
        this.pitchAngleRotations / other,
        this.yawAngleRotations / other,
        this.rollAngleRotations / other
    )
    operator fun div(other: Int): Rotation3d = Rotation3d(
        this.pitchAngleRotations / other,
        this.yawAngleRotations / other,
        this.rollAngleRotations / other
    )
}

class AngularVelocity private constructor(private val rps: Double) {
    companion object {
        fun fromRPS(rps: Double): AngularVelocity = AngularVelocity(rps)

        fun fromRPM(rpm: Double): AngularVelocity = AngularVelocity(rpm / 60)

        fun fromRadPS(radPS: Double): AngularVelocity = AngularVelocity(radPS / (2 * PI))

        fun fromDegPS(degPS: Double): AngularVelocity = AngularVelocity(degPS / 360)
    }

    val asRPS get() = rps
    val asRPM get() = rps * 60
    val asRadPS get() = rps * 2 * PI
    val asDegPS get() = rps * 360

    operator fun plus(other: AngularVelocity): AngularVelocity = AngularVelocity(this.rps + other.rps)
    operator fun minus(other: AngularVelocity): AngularVelocity = AngularVelocity(this.rps - other.rps)
    operator fun times(other: Double): AngularVelocity = AngularVelocity(this.rps * other)
    operator fun times(other: Int): AngularVelocity = AngularVelocity(this.rps * other)
    operator fun div(other: Double): AngularVelocity = AngularVelocity(this.rps / other)
    operator fun div(other: Int): AngularVelocity = AngularVelocity(this.rps / other)
}

class Translation2d(val x: Double, val y: Double) {
    constructor(length: Double, angle: Rotation2d): this(
        length * cos(angle.asRadians),
        length * sin(angle.asRadians)
    )

    val rotation: Rotation2d get() {
        val theta = Rotation2d.fromRadians(tanh(y.absoluteValue / x.absoluteValue))

        return if (x >= 0 && y >= 0) {
            theta
        } else if (x <= 0 && y >= 0) {
            Rotation2d.fromDegrees(180.0) - theta
        } else if (x <= 0 && y <= 0) {
            Rotation2d.fromDegrees(180.0) + theta
        } else {
            Rotation2d.fromDegrees(360.0) - theta
        }
    }

    operator fun plus(other: Translation2d): Translation2d = Translation2d(this.x + other.x, this.y + other.y)
    operator fun minus(other: Translation2d): Translation2d = Translation2d(this.x - other.x, this.y - other.y)
    operator fun times(other: Double): Translation2d = Translation2d(this.x * other, this.y * other)
    operator fun times(other: Int): Translation2d = Translation2d(this.x * other, this.y * other)
    operator fun div(other: Double): Translation2d = Translation2d(this.x / other, this.y / other)
    operator fun div(other: Int): Translation2d = Translation2d(this.x / other, this.y / other)
}

class Translation3d(val x: Double, val y: Double, val z: Double) {
    operator fun plus(other: Translation3d): Translation3d = Translation3d(this.x + other.x, this.y + other.y, this.z + other.z)
    operator fun minus(other: Translation3d): Translation3d = Translation3d(this.x - other.x, this.y - other.y, this.z - other.z)
    operator fun times(other: Double): Translation3d = Translation3d(this.x * other, this.y * other, this.z * other)
    operator fun times(other: Int): Translation3d = Translation3d(this.x * other, this.y * other, this.z * other)
    operator fun div(other: Double): Translation3d = Translation3d(this.x / other, this.y / other, this.z / other)
    operator fun div(other: Int): Translation3d = Translation3d(this.x / other, this.y / other, this.z / other)
}

class Pose2d(val translation2d: Translation2d, val rotation3d: Rotation3d) {

}