package org.firstinspires.ftc.teamcode.subsystems.limelight

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object Vision : Subsystem {
    lateinit var limelight: Limelight3A

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")

        limelight.pipelineSwitch(3)


        limelight.start()
    }







}