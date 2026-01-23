package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Spindexer : Subsystem {

    @JvmField
    var target = 0.0

    @JvmField
    var posPIDCoefficients = PIDCoefficients(0.0, 0.0, 0.0)

    val spindexer = MotorEx("spindexer").brakeMode()
    private val runtime = ElapsedTime()

    val controlSystem = controlSystem {
        posPid(Turret.posPIDCoefficients)
    }
}