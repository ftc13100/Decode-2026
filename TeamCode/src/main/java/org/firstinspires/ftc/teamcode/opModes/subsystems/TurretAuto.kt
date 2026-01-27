package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

@Configurable
object TurretAuto : Subsystem {
    private lateinit var turret1: Servo
    private lateinit var turret2: Servo
    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap.get(Servo::class.java, "gateLeft")
        turret2 = ActiveOpMode.hardwareMap.get(Servo::class.java, "gateRight")
    }

    val min = InstantCommand {
        turret1.position = 0.0
        turret2.position = 0.0
    }

    val max = InstantCommand {
        turret1.position = 1.0
        turret2.position = 1.0
    }

//    val toLeft = InstantCommand {
//        turret1.position = //blue shoot
//        turret2.position = //blue shoot
//    }

//    val toRight = InstantCommand {
//        turret1.position = //red shoot
//        turret2.position = //red shoot
//
//    }
}

