package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object NewTurret : Subsystem {
    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    var turretPosition = 0.0
        set(value) {
            turret1.position = value
            turret2.position = value
            field = value
        }
    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap["turret1"] as Servo
        turret2 = ActiveOpMode.hardwareMap["turret1"] as Servo

        turretPosition = 0.0
    }

    val increment = InstantCommand {
        turretPosition += 0.001
    }

    val decrement = InstantCommand {
        turretPosition -= 0.001
    }
}