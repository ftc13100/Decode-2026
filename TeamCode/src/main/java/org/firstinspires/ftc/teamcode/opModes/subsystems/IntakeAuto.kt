package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

object IntakeAuto : Subsystem {

    val intake = MotorEx("intake").brakeMode()

    var isRunning = false
    const val CURRENT_THRESHOLD = 7000.0

    val spinFast =
            SetPower(intake, 1.0)
            .requires(this)

    val spinStop =
            SetPower(intake, 0.0)
            .requires(this)
}

