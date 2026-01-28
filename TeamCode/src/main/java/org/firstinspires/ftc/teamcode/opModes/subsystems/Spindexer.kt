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

    val spindexer = MotorEx("spindexer").brakeMode()
    lateinit var color0: NormalizedColorSensor
    lateinit var color1: NormalizedColorSensor
    lateinit var color2: NormalizedColorSensor

    private val runtime = ElapsedTime()

    val spinAngle: Double
        get() = (360.0 / 384.5) * spindexer.currentPosition

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

// instead of periodic?
//    override val defaultCommand: Command = LambdaCommand()
//        .setUpdate {
//            spindexer.power = controlSystem.calculate(spindexer.state)
//        }
//        .requires(this)

    override fun periodic() {
        spindexer.power = controlSystem.calculate(spindexer.state)
    }

    // tuning only
    fun spin() {
        controlSystem.goal = KineticState(position = target)
        spindexer.power = controlSystem.calculate(spindexer.state)
    }

    //For shot
    val spinShot = SetPower(spindexer,-1.0).requires(this)

    val stopShot = SetPower(spindexer,0.0).requires(this)

    fun angleToTicks(angle : Double): Double {
        val ticks = angle * 384.5/360
        return ticks
    }

    fun ticksToAngle(ticks : Double): Double {
        val angle = ticks * 384.5/360
        return angle
    }

    val index0 = RunToPosition(controlSystem, 0.0)

    val index1 = RunToPosition(controlSystem, angleToTicks(120.0))

    val index2 = RunToPosition(controlSystem, angleToTicks(240.0))

    enum class SpindexerColor { PURPLE, GREEN, EMPTY }

    fun detectColorRGB(sensor: NormalizedColorSensor): SpindexerColor {
        val colors = sensor.normalizedColors

        // 1. Proximity Check: If Alpha is very low, the slot is empty
        if (colors.alpha < 0.1) return SpindexerColor.EMPTY

        // 2. Ratio Logic: Compare color intensities
        return when {
            // If Green is the dominant color
            colors.green > colors.red && colors.green > colors.blue -> {
                SpindexerColor.GREEN
            }
            // If Blue is dominant (Purple elements often register as high Blue)
            colors.blue > colors.red && colors.blue > colors.green -> {
                SpindexerColor.PURPLE
            }
            else -> SpindexerColor.EMPTY
        }
    }

    // color sensor stuff
//    fun getColor(): DoubleArray {
//        val raw = intArrayOf(color0.red(), color0.green(), color0.blue())
//        val sum = (raw[0] + raw[1] + raw[2]).toDouble()
//        return doubleArrayOf(raw[0] / sum, raw[1] / sum, raw[2] / sum)
//        }

    val dexing = intArrayOf(-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 2, 0, 1,
        1, 2, 0, 0, 1, 2, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 2, 0, 1, 1, 2, 0, 0,
        1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
        0, 0, 1, 0, 0, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 1, 2, 0, 0, 1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1,
        2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0,
        1, 1, 2, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0)

    override fun initialize() {
        color0 = hardwareMap.get(NormalizedColorSensor::class.java, "color0")
        color0.gain = 10.0f
        color1 = hardwareMap.get(NormalizedColorSensor::class.java, "color1")
        color1.gain = 10.0f
        color2 = hardwareMap.get(NormalizedColorSensor::class.java, "color2")
        color2.gain = 10.0f
    }

}