package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode()

    var isRunning = false
    const val CURRENT_THRESHOLD = 7000.0

    val spinFastAuto =
        SetPower(intake, 1.0)
            .requires(this)

    val spinStopAuto =
        SetPower(intake, 0.0)
            .requires(this)

    val spinFast =
        SequentialGroup(
        SetPower(intake, 1.0),
            InstantCommand { isRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { isRunning = false }
        )
            .requires(this)

    val spinStop =
        SequentialGroup(
        SetPower(intake, 0.0),
            InstantCommand { isRunning = false }
        )
            .requires(this)

    val spinSlowSpeed =
        SequentialGroup(
            SetPower(intake, 0.8),
            InstantCommand { isRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { isRunning = false }
        )
            .requires(this)

    val spinReverse =
        SequentialGroup(
            SetPower(intake, -0.7),
            InstantCommand { isRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD },
            SetPower(intake, 0.0),
            InstantCommand { isRunning = false }
        )
            .requires(this)
}

