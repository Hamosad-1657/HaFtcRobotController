package org.firstinspires.ftc.teamcode.subsystems.mecanum

import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Translation2d
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sqrt
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants

data class ChassisSpeeds(val translation: Translation2d, val omega: AngularVelocity)

object MecanumKinematics {
    // FL, BR, FR, BL
    private val moveRightWheelVelocityRelations: List<Double> = listOf(
        1.0,
        1.0,
        -1.0,
        -1.0,
    )

    private val moveForwardWheelVelocityRelations: List<Double> = listOf(
        1.0,
        1.0,
        1.0,
        1.0,
    )

    private val clockwiseRotationWheelVelocityRelations: List<Double> = listOf(
        1.0,
        -1.0,
        -1.0,
        1.0,
    )

    private fun factorSpeeds(unfactoredSpeeds: List<AngularVelocity>): List<AngularVelocity> {
        var highestAngularVelocity: AngularVelocity = AngularVelocity.fromRPS(0.0)
        for (speed in unfactoredSpeeds) {
            if (speed.asRPS.absoluteValue > highestAngularVelocity.asRPS.absoluteValue) {
                highestAngularVelocity = speed
            }
        }
        if (highestAngularVelocity.asRPS.absoluteValue > Constants.MAX_ROTATION_SPEED.asRPS) {
            val factor =
                Constants.MAX_ROTATION_SPEED.asRPS / highestAngularVelocity.asRPS.absoluteValue
            val factoredSpeeds: MutableList<AngularVelocity> = mutableListOf()
            for (speed in unfactoredSpeeds) {
                factoredSpeeds.add(speed * factor)
            }
            return factoredSpeeds.toList()
        }
        return unfactoredSpeeds.toList()
    }

    fun translationToMotorVelocities(speed: Translation2d): List<AngularVelocity> {
        val angularVelocityX = AngularVelocity.fromRPS(speed.x / (sqrt(2.0) * PI * Constants.WHEEL_CIRCUMFERENCE.asMeters))
        val angularVelocityY = AngularVelocity.fromRPS(speed.y / (sqrt(2.0) * PI * Constants.WHEEL_CIRCUMFERENCE.asMeters))

        val unfactoredSpeeds: MutableList<AngularVelocity> = mutableListOf()

        for (i in 0..3) {
            unfactoredSpeeds.add(angularVelocityX * moveRightWheelVelocityRelations[i] + angularVelocityY * moveForwardWheelVelocityRelations[i])
        }

        return factorSpeeds(unfactoredSpeeds.toList())
    }

    /** [angularVelocity] is clockwise positive. */
    fun angularVelocityToMotorVelocities(angularVelocity: AngularVelocity): List<AngularVelocity> {
        val angularVelocity = AngularVelocity.fromRPS(
            angularVelocity.asRPS * Constants.CHASSIS_CIRCUMFERENCE.asMeters / Constants.WHEEL_CIRCUMFERENCE.asMeters
        )

        val speeds: MutableList<AngularVelocity> = mutableListOf()
        for (relation in clockwiseRotationWheelVelocityRelations) {
            speeds.add(angularVelocity * relation)
        }
        return speeds.toList()
    }

    fun chassisSpeedsToMotorVelocities(chassisSpeeds: ChassisSpeeds): List<AngularVelocity> {
        val translation = translationToMotorVelocities(chassisSpeeds.translation)
        val rotation = angularVelocityToMotorVelocities(chassisSpeeds.omega)
        val combinedUnfactored: MutableList<AngularVelocity> = mutableListOf()
        for (i in 0..3) {
            combinedUnfactored.add(translation[i] + rotation[i])
        }

        return factorSpeeds(combinedUnfactored)
    }
}