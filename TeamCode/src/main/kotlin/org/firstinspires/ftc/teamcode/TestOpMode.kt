package org.firstinspires.ftc.teamcode

import com.hamosad.lib.commands.Subsystem
import com.hamosad.lib.opModes.CommandOpModeTeleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem

@TeleOp
class TestOpMode: CommandOpModeTeleop() {
    override var subsystemsToUse: List<Subsystem> = listOf(MecanumSubsystem)

    override fun configureBindings() {

    }

    override fun configureDefaultCommands() {

    }
}