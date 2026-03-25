package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.time.Duration.Companion.seconds

object Lift : Subsystem {

    private lateinit var ptoLeft: Servo
    private lateinit var ptoRight: Servo

    val backLeftMotor = MotorEx("backLeft").brakeMode()
    val backRightMotor = MotorEx("backRight").brakeMode()

    val LiftTimer = ElapsedTime()

    var isRunning = false
    const val CURRENT_THRESHOLD = 12000.0

    override fun initialize() {
        ptoLeft = ActiveOpMode.hardwareMap.get(Servo::class.java, "ptoLeft")
        ptoRight = ActiveOpMode.hardwareMap.get(Servo::class.java, "ptoRight")
    }

    val pto_lift = InstantCommand {
        ptoLeft.position = 0.45
        ptoRight.position = 0.55
    }

    val pto_drive = InstantCommand {
        ptoLeft.position = 0.5
        ptoRight.position = 0.5
    }

    fun toAngleRight(angle: Double) =
        InstantCommand {
            ptoRight.position = angle
        }

    fun toAngleLeft( angle: Double) =
        InstantCommand {
            ptoLeft.position = angle
        }

    val lift_Motors =
        SequentialGroup(
            InstantCommand { isRunning = true },
            InstantCommand { LiftTimer.reset() },
            ParallelGroup(
            SetPower(backRightMotor, 1.0),
            SetPower(backLeftMotor,1.0)
            ),
            WaitUntil { backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD || LiftTimer.seconds() > 8.0 },
            ParallelGroup(
            SetPower(backRightMotor, 0.0),
            SetPower(backLeftMotor, 0.0)
            ),
            InstantCommand { isRunning = false }
        )


    val motorsOn = InstantCommand {
        backLeftMotor.power = -1.0
        backRightMotor.power = 1.0
    }

    val motorsOff = InstantCommand {
        backLeftMotor.power = 0.0
        backRightMotor.power = 0.0
    }

    val full_Lift = SequentialGroup(
        pto_lift,
        Delay(1.seconds),
        lift_Motors
    )

}

