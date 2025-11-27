package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.commands.runOnce
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.hamosad.lib.components.Controllers.HaCommandController
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.angularVelocityDriveCommand
//import org.firstinspires.ftc.teamcode.commands.testMotorsCommand
//import org.firstinspires.ftc.teamcode.commands.testPIDCommand
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

@TeleOp
class TestOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)
    val controller = HaCommandController({ super.gamepad1 }, 0.03, 1)

    override fun configureDefaultCommands() {
        MecanumSubsystem.defaultCommand = MecanumSubsystem.angularVelocityDriveCommand(
            true,
            { controller.getLeftX() },
            { controller.getLeftY() },
            { controller.getRightX() },
        )
    }

    override fun configureBindings() {
        controller.options().onTrue(MecanumSubsystem.runOnce { MecanumSubsystem.resetGyro() })
//        controller.triangle().whileTrue(MecanumSubsystem.testMotorsCommand(0))
//        controller.circle().whileTrue(MecanumSubsystem.testMotorsCommand(1))
//        controller.cross().whileTrue(MecanumSubsystem.testMotorsCommand(2))
//        controller.square().whileTrue(MecanumSubsystem.testMotorsCommand(3))
//
//        controller.dpadUp().whileTrue(MecanumSubsystem.testPIDCommand())
    }
}

