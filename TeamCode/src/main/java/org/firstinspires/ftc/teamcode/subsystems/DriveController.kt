package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.extensions.pedro.TurnBy
import dev.nextftc.extensions.pedro.TurnTo
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.driving.Drivetrain
import kotlin.math.atan2

object DriveController: SubsystemGroup(Drivetrain) {
    override fun initialize() {
        defaultCommand.scalar = 0.9
        defaultCommand.requires(this)
    }

    override val defaultCommand = PedroDriverControlled(
        -Gamepads.gamepad1.leftStickY,
        Gamepads.gamepad1.leftStickX,
        Gamepads.gamepad1.rightStickX,
    )

    fun pointToRed(pose: Pose) : Command {
        val redGoal = Pose()
        val diff = redGoal - pose

        return TurnTo(
            atan2(diff.y, diff.x).rad.normalized
        ).requires(this)
    }

    fun pointToBlue(pose: Pose) : Command {
        val blueGoal = Pose()
        val diff = blueGoal - pose

        return TurnTo(
            atan2(diff.y, diff.x).rad.normalized
        ).requires(this)
    }
}