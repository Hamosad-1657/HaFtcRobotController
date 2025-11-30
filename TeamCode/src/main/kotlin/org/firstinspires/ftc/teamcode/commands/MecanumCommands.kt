package org.firstinspires.ftc.teamcode.commands

import com.hamosad.lib.commands.Command
import com.hamosad.lib.commands.runCommand
import com.hamosad.lib.math.AngularVelocity
import com.hamosad.lib.math.Translation2d
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.pedropathing.PedroConstants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.ChassisSpeeds
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants as Constants
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

fun MecanumSubsystem.angularVelocityDriveCommand(
    fieldRelative: Boolean,
    leftJoyX: () -> Double,
    leftJoyY: () -> Double,
    rightJoyX: () -> Double
) = MecanumSubsystem.runCommand {
    drive(fieldRelative, ChassisSpeeds(
        Translation2d(leftJoyX() * Constants.MAX_CHASSIS_SPEED, leftJoyY() * Constants.MAX_CHASSIS_SPEED),
        AngularVelocity.fromRPS(rightJoyX() * Constants.MAX_CHASSIS_ANGULAR_VELOCITY.asRPS),
    ))
}

//fun MecanumSubsystem.testMotorsCommand(motor: Int): Command = MecanumSubsystem.runCommand {
//    testMotors(motor)
//}
//
//fun MecanumSubsystem.testPIDCommand(): Command = MecanumSubsystem.runCommand {
//    testPID()
//}

//fun MecanumSubsystem.followPathCommand(path: Path, hardwareMap: HardwareMap): Command {
//    val follower = PedroConstants.createFollower(hardwareMap)
//}