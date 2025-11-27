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

    var targetPosition = 0.0

    fun update() : InstantCommand {
        // Set the servo position directly
        return InstantCommand { servo.position = targetPosition }
    }


    val angle_up = InstantCommand {
        servo.position = 0.5
    }

    val angle_middle = InstantCommand {
        servo.position = 0.6
    }

    val angle_down = InstantCommand {
        servo.position = 0.7
    }
}
