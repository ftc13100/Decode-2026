package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode().reversed()

    var intakeRunning = false
    const val CURRENT_THRESHOLD = 5000.0

    val spinFast =
        SequentialGroup(
        SetPower(intake, -0.9),
            InstantCommand { intakeRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)

    val spinStop =
        SequentialGroup(
        SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)

    val spinSlowSpeed =
        SequentialGroup(
            SetPower(intake, 0.7),
            InstantCommand { intakeRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)

    val spinReverse =
        SequentialGroup(
            SetPower(intake, 0.7),
            InstantCommand { intakeRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)
}

