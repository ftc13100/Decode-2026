package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object ShooterAngle: Subsystem {
    private lateinit var servo : Servo
    override fun initialize() {
        servo = ActiveOpMode.hardwareMap.get(Servo::class.java, "angle")
    }

    var targetPosition: Double = 0.0

    fun update() {
        // Set the servo position directly
        servo.position = targetPosition
    }


    val angle_up = InstantCommand {
        servo.position = 0.0
    }

    val angle_middle = InstantCommand {
        servo.position = 0.1
    }

    val angle_down = InstantCommand {
        servo.position = 0.2
    }
}
