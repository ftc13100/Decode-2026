package org.firstinspires.ftc.teamcode.opModes.subsystems

import android.graphics.Color
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.time.Instant
import kotlin.math.abs

@Configurable
object Spindexer : Subsystem {
    @JvmField var target = 0.0
    // Position PID used for indexing
    @JvmField var posPIDCoefficients = PIDCoefficients(0.01, 0.0, 0.0002)

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    val spinAngle: Double
        get() = (360.0 / 384.5) * spindexer.currentPosition

    val spindexer = MotorEx("spindexer").brakeMode()
    lateinit var color0: NormalizedColorSensor
    lateinit var color1: NormalizedColorSensor
    lateinit var color2: NormalizedColorSensor



    enum class State { PID, MANUAL }

    var state = State.PID

    override fun periodic() {
        when (state) {
            State.PID -> {
                spindexer.power = controlSystem.calculate(spindexer.state)
                detectColorRGB(color0)
                detectColorRGB(color1)
                detectColorRGB(color2)
            }
            State.MANUAL -> {
                return
            }
        }
    }

    fun forwardOnlyTarget(angleDeg: Double): Double {
        val targetInRev = angleToTicks(angleDeg)
        val currentRev = kotlin.math.floor(spindexer.currentPosition / 384.5)
        var newTarget = currentRev * 384.5 + targetInRev
        if (newTarget <= spindexer.currentPosition) {
            newTarget += 384.5
        }
        return newTarget
    }

    // Indexing
    // PID state: schedules RunToPosition
    val index0 = InstantCommand({ state = State.PID })
        .then(RunToPosition(controlSystem, forwardOnlyTarget(0.0)))
        .requires(this)

    val index1 = InstantCommand({ state = State.PID })
        .then(RunToPosition(controlSystem, forwardOnlyTarget(120.0)))
        .requires(this)

    val index2 = InstantCommand({ state = State.PID })
        .then(RunToPosition(controlSystem, forwardOnlyTarget(240.0)))
        .requires(this)

    // manual: periodic stops PID
    val spinShot = InstantCommand({ state = State.MANUAL })
        .then(SetPower(spindexer, -1.0))
        .requires(this)

    val stopShot = InstantCommand({ state = State.MANUAL })
        .then(SetPower(spindexer, 0.0))
        .requires(this)

    val autoIndex = InstantCommand({ state = State.PID
        when (desiredIndex()) {
            0 -> index0()
            1 -> index1()
            2 -> index2()
        }
    }).requires(this)


    // tuning only
    fun spin() {
        controlSystem.goal = KineticState(position = target)
        spindexer.power = controlSystem.calculate(spindexer.state)
    }

    fun angleToTicks(angle : Double): Double {
        val ticks = angle * 384.5/360
        return ticks
    }

    fun ticksToAngle(ticks : Double): Double {
        val angle = ticks * 384.5/360
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

    private fun colorToDigit(color: SpindexerColor): Int =
        when (color) {
            SpindexerColor.EMPTY -> 0
            SpindexerColor.GREEN -> 1
            SpindexerColor.PURPLE -> 2
        }

    fun computeDexIndex(): Int {
        val b0 = colorToDigit(detectColorRGB(color0))
        val b1 = colorToDigit(detectColorRGB(color1))
        val b2 = colorToDigit(detectColorRGB(color2))
        val b3 = 0
        //Shoot command button need to figure out how to implement this,
        // probably done in main teleop?
        // however would probaby also mean this function would have to be done in main
        val b4 = 0 //Motif

        return b0 * 81 + b1 * 27 + b2 * 9 + b3 * 3 + b4
    }

    fun desiredIndex(): Int =
        dexing[computeDexIndex()]

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
        color0 = hardwareMap.get(NormalizedColorSensor::class.java, "color0")
        color0.gain = 10.0f
        color1 = hardwareMap.get(NormalizedColorSensor::class.java, "color1")
        color1.gain = 10.0f
        color2 = hardwareMap.get(NormalizedColorSensor::class.java, "color2")
        color2.gain = 10.0f
    }

}