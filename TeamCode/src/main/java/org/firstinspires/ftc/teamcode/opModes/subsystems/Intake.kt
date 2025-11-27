package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.controller

object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode()


    val autoFast =
        SetPower(intake, 1.0)

    val autoStop =
        SetPower(intake, 0.0)

    fun spinSlowSpeed() = InstantCommand {
        intake.power = 0.7
    }

    fun spinFastSpeed() = InstantCommand {
        intake.power = 1.0
    }

    fun stop() = InstantCommand {
        intake.power = 0.0
    }

}

