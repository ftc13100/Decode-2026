package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object Gate : Subsystem {
    private lateinit var servoLeft: Servo
    private lateinit var servoRight: Servo

    override fun initialize() {
        servoLeft = ActiveOpMode.hardwareMap.get(Servo::class.java, "gateLeft")
        servoRight = ActiveOpMode.hardwareMap.get(Servo::class.java, "gateRight")
    }

    val gate_in = InstantCommand {
        servoLeft.position = 0.0
        servoRight.position = 1.0
    }

    val gate_stop = InstantCommand {
        servoLeft.position = 0.5
        servoRight.position = 0.5
    }

    val gate_out = InstantCommand {
        servoLeft.position = 1.0
        servoRight.position = 0.0

    }

}

