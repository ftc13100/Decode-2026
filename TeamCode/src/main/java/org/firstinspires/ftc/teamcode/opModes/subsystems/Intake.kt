package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.controller

object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode()


    val spinFast =
        SetPower(intake, 1.0)

    val spinStop =
        SetPower(intake, 0.0)

    val spinSlowSpeed =
        SetPower(intake, 0.7,)



    val spinReverse =
        SetPower(intake, -0.7,)



}

