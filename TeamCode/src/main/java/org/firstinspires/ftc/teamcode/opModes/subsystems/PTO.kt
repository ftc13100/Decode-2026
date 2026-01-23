package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object PTO : Subsystem {

    private lateinit var pto1: Servo
    private lateinit var pto2: Servo

    override fun initialize() {
        pto1 = ActiveOpMode.hardwareMap.get(Servo::class.java, "pto1")
        pto2 = ActiveOpMode.hardwareMap.get(Servo::class.java, "pto2")
    }

    val pto_lift = InstantCommand {
        pto1.position = 0.0
        pto2.position = 0.0
    }

    val pto_drive = InstantCommand {
        pto1.position = 0.5
        pto2.position = 0.5
    }

}

