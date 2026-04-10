package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer.spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer.state
import kotlin.time.Duration.Companion.seconds

@Configurable
object Intake : Subsystem {

    val intake = MotorEx("intake").brakeMode().reversed()

    var intakeRunning = false
    @JvmField var CURRENT_THRESHOLD_FAST = 8000.0
    @JvmField var CURRENT_THRESHOLD_SLOW = 4000.0

    val spinFast =
        SequentialGroup(
        SetPower(intake, -1.0),
            InstantCommand { intakeRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD_FAST },
            SetPower(intake, 0.4),
            Delay(0.3.seconds),
            SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)

    val spinStop = InstantCommand {
        intakeRunning = false
        intake.power = 0.0
    }
        .requires(this)

    val spinSlowSpeed = {
        SequentialGroup(
            SetPower(intake, -0.5),
            InstantCommand { intakeRunning = true },
            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD_SLOW },
            SetPower(intake, 0.4),
            Delay(0.3.seconds),
            SetPower(intake, 0.0),
            InstantCommand { intakeRunning = false }
        )
            .requires(this)
    }

//    val spinReverse =
//        SequentialGroup(
//            SetPower(intake, 0.7),
//            InstantCommand { intakeRunning = true },
//            WaitUntil { intake.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD_FAST },
//            SetPower(intake, 0.4),
//            Delay(0.3.seconds),
//            SetPower(intake, 0.0),
//            InstantCommand { intakeRunning = false }
//        )
//            .requires(this)

    val spinReverse = InstantCommand {
        intakeRunning = true
        intake.power = 0.7
    }
        .requires(this)

}

