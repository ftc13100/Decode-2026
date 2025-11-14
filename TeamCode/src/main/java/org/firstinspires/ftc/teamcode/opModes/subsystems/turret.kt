package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.shooter

//import dev.nextftc.hardware.powerable.SetPower


@Configurable
object turret : Subsystem {
    private val turret = MotorEx("turret").brakeMode()

    private val controlSystem = controlSystem {
        posPid(0.03, 0.01, 5.0)
    }

    fun spinRight(){
        turret.power = 0.7
    }
    fun spinLeft(){
        turret.power = -0.7
    }
    fun spinZero(){
        turret.power = 0.0
    }
    val toRight = RunToPosition(controlSystem, 300.0).requires(this)
    val toMiddle = RunToPosition(controlSystem, 0.0).requires(this)
    val toLeft = RunToPosition(controlSystem, -300.0).requires(this)

    override fun periodic() {
        turret.power = controlSystem.calculate(turret.state)
    }
}