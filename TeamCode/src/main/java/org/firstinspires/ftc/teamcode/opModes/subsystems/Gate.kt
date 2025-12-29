package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter.controller

object Gate : Subsystem {
    private lateinit var servo : Servo
    override fun initialize() {
        servo = ActiveOpMode.hardwareMap.get(Servo::class.java, "gate")
    }

    val gate_open = InstantCommand {
        servo.position = 0.375
    }

    val gate_close = InstantCommand {
        servo.position = 0.175
    }

}

