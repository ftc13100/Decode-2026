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
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

@Configurable
object Spindexer : Subsystem {
    @JvmField var target = 0.0
    // Position PID used for indexing
    @JvmField var posPIDCoefficients = PIDCoefficients(-0.0014, 0.0, -0.000035)

    //ID 23: PPG = 2
    //ID 22: PGP = 1
    //ID 21: GPP = 0

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    val tolerance = KineticState(20.0)

    val spinAngle: Double
        get() = (360.0 / (4000.0) * spindexer.currentPosition)

    val spindexer = MotorEx("spindexer").brakeMode()
    lateinit var color0: NormalizedColorSensor
    lateinit var color1: NormalizedColorSensor
    lateinit var color2: NormalizedColorSensor

    enum class State { PID, MANUAL }

    var state = State.PID

    override fun periodic() {
        when (state) {
            State.PID -> {
                spindexer.power = controlSystem.calculate(spindexer.state).coerceIn(-0.8,0.8)
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
        val currentRev = kotlin.math.floor(spindexer.currentPosition / (4000.0))
        var newTarget = currentRev * (4000.0) + targetInRev
        if (newTarget <= spindexer.currentPosition) {
            newTarget += (4000.0)
        }
        return newTarget
    }

    fun intakePos(): Double {
        val step = 1333.0
        return kotlin.math.round(spindexer.currentPosition / step) * step
    }

    val toIntakePos = LambdaCommand("toIntakePos")
        .setStart {
            state = State.PID
            controlSystem.goal =
                KineticState(intakePos())
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .requires(this)

    // only if needed
    val wiggle = LambdaCommand("wiggleUp")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(10.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        .then(
            LambdaCommand("wiggleBack")
                .setStart {
                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(10.0))
                }
                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
        )
        .requires(this)

    // Indexing
    // PID state: schedules RunToPosition
    val index0 = LambdaCommand("Index0Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(0.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        .then(
//            LambdaCommand("Index0Return")
//                .setStart {
//                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
//                }
//                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        )
        .requires(this)

    val index1 = LambdaCommand("Index1Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(120.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        .then(
//            LambdaCommand("Index1Return")
//                .setStart {
//                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
//                }
//                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        )
        .requires(this)

    val index2 = LambdaCommand("Index2Overshoot")
        .setStart {
            state = State.PID
            controlSystem.goal = KineticState(forwardOnlyTarget(240.0))
        }
        .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        .then(
//            LambdaCommand("Index2Return")
//                .setStart {
//                    controlSystem.goal = KineticState(spindexer.currentPosition-angleToTicks(120.0))
//                }
//                .setIsDone { controlSystem.isWithinTolerance(tolerance) }
//        )
        .requires(this)

    // manual: periodic stops PID
    val spinShot = InstantCommand {
        Intake.spinSlowSpeed()()
        state = State.MANUAL
        spindexer.power = 1.0
    }
        .requires(this)
    val spinShoot = SetPower(spindexer, 1.0)
    val stopshoot = SetPower(spindexer, 0.0)



    val spinIndex = InstantCommand {
        Intake.spinSlowSpeed()()
        state = State.MANUAL
        spindexer.power = -1.0
    }
        .requires(this)


    val stopShot = InstantCommand {
        state = State.MANUAL
        spindexer.power = 0.0
    }
        .requires(this)

    fun autoIndex(b3: Int) = InstantCommand {
        state = State.PID
        when (desiredIndex(b3, PoseStorage.motif)) {
            0 -> index0()
            1 -> index1()
            2 -> index2()
        }
    }.requires(this)

    // tuning only
    fun spin() {
        controlSystem.goal = KineticState(position = target)
        spindexer.power = controlSystem.calculate(spindexer.state)
    }

    fun angleToTicks(angle : Double): Double {
        val ticks = angle * (4000.0)/360
        return ticks
    }

    fun ticksToAngle(ticks : Double): Double {
        val angle = ticks * 360/(4000.0)
        return angle
    }

    enum class SpindexerColor { PURPLE, GREEN, EMPTY }

    fun detectColorRGB(sensor: NormalizedColorSensor): SpindexerColor {
        val colors = sensor.normalizedColors

        // Proximity, if Alpha low, slot empty
        if (colors.alpha < 0.15) return SpindexerColor.EMPTY

        return when {
            // If Green dominant
            colors.green > colors.red && colors.green > colors.blue -> {
                SpindexerColor.GREEN
            }
            // If Blue dominant (Purple)
            colors.blue > colors.red && colors.blue > colors.green -> {
                SpindexerColor.PURPLE
            }
            else -> SpindexerColor.EMPTY
        }
    }

    val intakePos = RunToPosition(controlSystem, 0.0).requires(this)

    private fun colorToDigit(color: SpindexerColor): Int =
        when (color) {
            SpindexerColor.EMPTY -> 0
            SpindexerColor.GREEN -> 2
            SpindexerColor.PURPLE -> 1
        }

    fun computeDexIndex(b3: Int, b4: Int): Int {
        val b0 = colorToDigit(detectColorRGB(color0))
        val b1 = colorToDigit(detectColorRGB(color1))
        val b2 = colorToDigit(detectColorRGB(color2))
        return b0 * 81 + b1 * 27 + b2 * 9 + b3 * 3 + b4
    }

    fun desiredIndex(b3: Int, b4: Int): Int =
        dexing[computeDexIndex(b3, b4)]

    val dexing = intArrayOf(-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 2, 0, 1,
        1, 2, 0, 0, 1, 2, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 2, 0, 1, 1, 2, 0, 0,
        1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
        0, 0, 1, 0, 0, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 1, 2, 0, 0, 1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1,
        2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0,
        1, 1, 2, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0)

    val isFull: Boolean
        get() {
            var purples = 0
            var greens = 0
            val sensors = listOf(color0, color1, color2)
            for (sensor in sensors) {
                val detected = detectColorRGB(sensor)
                if (detected == SpindexerColor.PURPLE) purples++
                if (detected == SpindexerColor.GREEN) greens++
            }
            return purples == 2 && greens == 1
        }

    override fun initialize() {
        color0 = hardwareMap.get(NormalizedColorSensor::class.java, "cs0")
        color0.gain = 12.0f
        color1 = hardwareMap.get(NormalizedColorSensor::class.java, "cs1")
        color1.gain = 12.0f
        color2 = hardwareMap.get(NormalizedColorSensor::class.java, "cs2")
        color2.gain = 12.0f
    }
}