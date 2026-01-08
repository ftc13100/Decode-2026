package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode()


    val spinFast =
        SetPower(intake, 1.0)

    val spinStop =
        SetPower(intake, 0.0)

    val spinSlowSpeed =
        SetPower(intake, 0.8)


    val spinReverse =
        SetPower(intake, -0.7)


}

