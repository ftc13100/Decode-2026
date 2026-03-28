package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake.intake

@Configurable
object Spindexer : Subsystem {
    @JvmField var target = 0.0
    // Position PID used for indexing
    @JvmField var posPIDCoefficients = PIDCoefficients(-0.0005, 0.0, -0.00002)

    //ID 23: PPG = 2
    //ID 22: PGP = 1
    //ID 21: GPP = 0

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    val tolerance = KineticState(10.0)

    val spinAngle: Double
        get() = (360.0 / (4000.0 * 5/2)) * spindexer.currentPosition

    val spindexer = MotorEx("spindexer").brakeMode()
    lateinit var color0: NormalizedColorSensor
    lateinit var color1: NormalizedColorSensor
    lateinit var color2: NormalizedColorSensor

    enum class State { PID, MANUAL }

    var state = State.PID

    override fun periodic() {
        when (state) {
            State.PID -> {
                spindexer.power = controlSystem.calculate(spindexer.state).coerceIn(-1.0, 1.0)
//                detectColorRGB(color0)
//                detectColorRGB(color1)
//                detectColorRGB(color2)
            }
            State.MANUAL -> {
                return
            }
        }
    }

    fun forwardOnlyTarget(angleDeg: Double): Double {
        val targetInRev = angleToTicks(angleDeg)
        val currentRev = kotlin.math.floor(spindexer.currentPosition / (4000.0 * 5/2))
        var newTarget = currentRev * (4000.0 * 5/2) + targetInRev
        if (newTarget <= spindexer.currentPosition) {
            newTarget += (4000.0 * 5/2)
        }
        return newTarget
    }

    // only if needed
    val wiggle = LambdaCommand("wiggleUp")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(25.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .then(
            LambdaCommand("wiggleBack")
                .setStart {
                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(25.0))
                }
                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        )
        .requires(this)

    val resetIndex0 = LambdaCommand("SpindexerPIDCommand")
        .setStart {
            state = State.PID
            controlSystem.goal =
                KineticState(forwardOnlyTarget(0.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .requires(this)

    // Indexing
    // PID state: schedules RunToPosition
    val index0 = LambdaCommand("Index0Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(120.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .then(
            LambdaCommand("Index0Return")
                .setStart {
                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
                }
                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        )
        .requires(this)

    val index1 = LambdaCommand("Index1Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(240.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .then(
            LambdaCommand("Index1Return")
                .setStart {
                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
                }
                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        )
        .requires(this)

    val index2 = LambdaCommand("Index2Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(360.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .then(
            LambdaCommand("Index2Return")
                .setStart {
                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
                }
                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        )
        .requires(this)

    // manual: periodic stops PID
    val shootAuto=
        SetPower(spindexer, 1.0)
            .requires(this)
    val shootStop=
        SetPower(spindexer, 0.0)
            .requires(this)


    fun angleToTicks(angle : Double): Double {
        val ticks = angle * (4000.0 * 5/2)/360
        return ticks
    }

   }