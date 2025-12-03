package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.controller
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.shooterActive
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.shooterReady


@Configurable
object Turret : Subsystem {
    @JvmField var target = 0.0
    @JvmField var PIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)


    val turret = MotorEx("turret").brakeMode()

    val controlSystem = controlSystem {
        posPid(0.025, 0.00, 0.0)
    }

    fun spinRight(){
        turret.power = 0.3
    }

    fun spinLeft(){
        turret.power = -0.3
    }

    fun spinZero() {
        turret.power = 0.0
    }

    fun spinToPos(pos: Double) =
        InstantCommand{ RunToPosition(controlSystem, pos) }


//    val toRight = RunToPosition(controlSystem, 300.0).requires(this)
//    val toMiddle = RunToPosition(controlSystem, 0.0).requires(this)
//    val toLeft = RunToPosition(controlSystem, -300.0).requires(this)

//    fun moveToTag (tagPos: Double) =
//        if (tagPos < 300 && tagPos > -300) {
//            RunToPosition(controlSystem, tagPos).requires(this)
//        } else {
//            InstantCommand {
//                spinZero()
//            }
//        }
//
//    override fun periodic() {
//        turret.power = controlSystem.calculate(turret.state)
//    }
}