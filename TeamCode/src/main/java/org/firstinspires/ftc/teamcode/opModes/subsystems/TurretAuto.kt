package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object TurretAuto : Subsystem {
    @JvmField
    var target = 0.0

    @JvmField
    var posPIDCoefficients = PIDCoefficients(0.01, 0.0, 0.00013)
    val turret = MotorEx("turret").brakeMode().zeroed()

    override fun initialize() {
        turret.zero()
        PoseStorage.turretStartPos = turret.currentPosition
    }

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    val toMid = RunToPosition(controlSystem, 0.0).requires(this)
    val toLeft = RunToPosition(controlSystem, -527.0).requires(this)
    val toRight = RunToPosition(controlSystem, 527.0).requires(this)

    val toLeftMohit = RunToPosition(controlSystem, -245.0).requires(this)
    val toRightMohit = RunToPosition(controlSystem, 245.0).requires(this)

    override fun periodic() {
        turret.power = controlSystem.calculate(turret.state)
    }
}

