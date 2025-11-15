package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.delegates.LazyHardware
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.velPIDCoefficients

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