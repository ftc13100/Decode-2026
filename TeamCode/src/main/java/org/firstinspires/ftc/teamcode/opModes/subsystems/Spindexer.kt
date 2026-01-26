package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import kotlin.math.abs

@Configurable
object Spindexer : Subsystem {
    @JvmField var target = 0.0
    // Position PID used for indexing
    @JvmField var posPIDCoefficients = PIDCoefficients(0.025, 0.0, 0.0015)

    val spindexer = MotorEx("spindexer").brakeMode().reversed()
//    lateinit var color0: RevColorSensorV3
//    lateinit var color1: RevColorSensorV3
//    lateinit var color2: RevColorSensorV3

    private val runtime = ElapsedTime()

    val spinAngle: Double
        get() = (360.0 / 384.5) * spindexer.currentPosition

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

//    override fun periodic() {
//        controlSystem.goal = KineticState(target)
//        spindexer.power = controlSystem.calculate(spindexer.state)
//    }

    fun spin() {
        controlSystem.goal = KineticState(position = target)
        spindexer.power = controlSystem.calculate(spindexer.state)
    }

    //For shot
    val spinShot = SetPower(spindexer,1.0)

    val stopShot = SetPower(spindexer,0.0)

    fun angleToTicks(angle : Double): Double {
        val ticks = angle * 384.5/360
        return ticks
    }

    fun index0() {
        RunToPosition(controlSystem, 0.0)
    }

    fun index1() {
        RunToPosition(controlSystem, angleToTicks(120.0))
    }

    fun index2() {
        RunToPosition(controlSystem, angleToTicks(240.0))
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
//        color0 = hardwareMap.get(RevColorSensorV3::class.java, "color0")
//        color1 = hardwareMap.get(RevColorSensorV3::class.java, "color1")
//        color2 = hardwareMap.get(RevColorSensorV3::class.java, "color2")
    }

}