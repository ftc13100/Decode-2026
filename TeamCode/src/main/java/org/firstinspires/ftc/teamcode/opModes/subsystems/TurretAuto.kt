package org.firstinspires.ftc.teamcode.opModes.subsystems.shooter

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object TurretAuto : Subsystem {
    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret1")
        turret2 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret2")

    }



    val toMid = InstantCommand {
        turret1.position = 0.5
    }
    val toMid2 = InstantCommand {
        turret2.position = 0.5
    }
    val toLeft = InstantCommand {
        turret1.position = 0.75
    }
    val toLeft2 = InstantCommand {
        turret2.position = 0.75
    }
    val toRight = InstantCommand {
        turret1.position = 0.25
    }
    val toRight2 = InstantCommand {
        turret2.position = 0.25
    }


}

