package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.delegates.LazyHardware
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.velPIDCoefficients
import kotlin.math.acos
import kotlin.math.pow

@Configurable
object Turret : Subsystem {
    @JvmField var posPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)
    val turret = MotorEx("turret")
        .brakeMode()
    private lateinit var limelight: Limelight3A

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(
            Limelight3A::class.java, "limelight"
        )
    }

    private var controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    fun spinRight(){
        turret.power = 0.7
    }

    fun spinLeft(){
        turret.power = -0.7
    }

    fun spinZero() {
        turret.power = 0.0
    }

    fun computeAngle (botPose: Pose): Command {
        val basketPose = Pose(12.0, 135.5)
        val diff = basketPose - botPose

        val c = basketPose.distanceFrom(botPose)
        val a = diff.x
        val b = diff.y

        val cosine = (a.pow(2) - b.pow(2) - c.pow(2)) / (-2 * b * c)

        val angle = acos(cosine)

        // Convert angles to ticks
        val ticks = 0.0 // Compute conversion

        return RunToPosition(controlSystem, ticks).requires(this)
    }

//    val toRight = RunToPosition(controlSystem, 300.0).requires(this)
    val toMiddle = RunToPosition(controlSystem, 0.0).requires(this)
//    val toLeft = RunToPosition(controlSystem, -300.0).requires(this)

//    fun moveToTag (tagPos: Double) =
//        if (tagPos < 300 && tagPos > -300) {
//            RunToPosition(controlSystem, tagPos).requires(this)
//        } else {
//            InstantCommand {
//                spinZero()
//            }
//        }

    override fun periodic() {
        controlSystem {
            posPid(posPIDCoefficients)
        }
        val result = limelight.latestResult
        if (result != null && result.isValid) {
            turret.power = controlSystem.calculate(
                KineticState(result.tx)
            )
        } else {
            turret.power = 0.0
        }
    }
}