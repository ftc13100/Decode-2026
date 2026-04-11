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
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret.setServoPos
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret.stopTracking
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret.targetAngleRobotRef
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object Lift : Subsystem {
    lateinit var ptoLeft: Servo
    lateinit var ptoRight: Servo
    val backLeftMotor = MotorEx("backLeft").brakeMode()
    val backRightMotor = MotorEx("backRight").brakeMode()
    val LiftTimer = ElapsedTime()
    var isRunning = false

    //Encoder
    private var leftStartPos = 0
    private var rightStartPos = 0

    const val SYNC_THRESHOLD = 400
    const val SYNC_RESUME = 100

    // Track which motor is paused for sync
    private var leftPaused  = false
    private var rightPaused = false

    const val CURRENT_THRESHOLD = 20000.0

    override fun initialize() {
        ptoLeft  = ActiveOpMode.hardwareMap.get(Servo::class.java, "ptoLeft")
        ptoRight = ActiveOpMode.hardwareMap.get(Servo::class.java, "ptoRight")
    }

//    fun toAngleRight(angle: Double) = InstantCommand { ptoRight.position = angle }
//    fun toAngleLeft (angle: Double) = InstantCommand { ptoLeft.position  = angle }

    val pto_lift = InstantCommand {
        ptoLeft.position  = 0.44
        ptoRight.position = 0.55
    }

    val pto_drive = InstantCommand {
        ptoLeft.position  = 0.477
        ptoRight.position = 0.525
    }

    fun toPosLeft(pos: Double) =
        InstantCommand {
            ptoLeft.position = pos

        }
    fun toPosRight(pos: Double) =
        InstantCommand {
            ptoRight.position = pos

        }

    // Call this right before the lift motors start spinning
    fun recordStartPositions() {
        leftStartPos  = backLeftMotor.motor.currentPosition
        rightStartPos = backRightMotor.motor.currentPosition
    }

    // Absolute ticks travelled since recordStartPositions() was called
    fun leftTravelTicks():  Int = abs(backLeftMotor.motor.currentPosition - leftStartPos)
    fun rightTravelTicks(): Int = abs(backRightMotor.motor.currentPosition - rightStartPos)

    // Call every loop while the lift is running.
    // If one side is more than SYNC_THRESHOLD ticks ahead of the other,
    // it is paused until the lagging side catches up within SYNC_RESUME ticks.
    fun syncLiftMotors() {
        if (!isRunning) return

        val leftTicks  = leftTravelTicks()
        val rightTicks = rightTravelTicks()
        val gap = leftTicks - rightTicks   // positive, left is ahead

        when {
            // Left is too far ahead, pause left, keep right running
            gap > SYNC_THRESHOLD && !leftPaused -> {
                backLeftMotor.power = 0.0
                backRightMotor.power = 1.0
                leftPaused  = true
                rightPaused = false
            }

            // Right is too far ahead, pause right, keep left running
            gap < -SYNC_THRESHOLD && !rightPaused -> {
                backRightMotor.power = 0.0
                backLeftMotor.power  = -1.0
                leftPaused  = false
                rightPaused = true
            }

            // Gap has closed enough, resume both
            (leftPaused || rightPaused) && abs(gap) <= SYNC_RESUME -> {
                backLeftMotor.power  = -1.0
                backRightMotor.power =  1.0
                leftPaused  = false
                rightPaused = false
            }

        }
    }

    val lift_Motors = SequentialGroup(
        InstantCommand { isRunning = true },
        InstantCommand { LiftTimer.reset() },
        // record encoder positions right before power is applied
        InstantCommand { recordStartPositions() },
        ParallelGroup(
            SetPower(backRightMotor,  1.0),
            SetPower(backLeftMotor,  -1.0)
        ),
        WaitUntil {
            backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD ||
                    LiftTimer.seconds() > 15.0 ||
                    backLeftMotor.motor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD
        },
        ParallelGroup(
            SetPower(backRightMotor, 0.0),
            SetPower(backLeftMotor,  0.0)
        ),
        InstantCommand {
            isRunning   = false
            leftPaused  = false
            rightPaused = false
        }
    )

    val motorsOn = InstantCommand {
        backLeftMotor.power  = -1.0
        backRightMotor.power =  1.0
    }

    val motorsOff = InstantCommand {
        backLeftMotor.power  = 0.0
        backRightMotor.power = 0.0
    }

    val full_Lift = SequentialGroup(
        pto_lift,
        Delay(0.5.seconds),
        lift_Motors
    )
}