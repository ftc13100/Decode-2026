package org.firstinspires.ftc.teamcode.opModes.teleOp.limelight

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.GreenCount

@TeleOp(name = "greencount")
class GreenCountOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(GreenCount)
        )
    }

    override fun onUpdate() {
        val result = GreenCount.limelight.latestResult

        telemetry.addData("Status", "Green counter active")
        telemetry.addData("Pipeline", 7)
        telemetry.addData("Detections", GreenCount.detectionCount)
        telemetry.addData("Final Total", GreenCount.totalBallCount)
        telemetry.addData("Limelight Has Target", result?.isValid ?: false)

        for (line in GreenCount.detectionBreakdown.take(10)) {
            telemetry.addLine(line)
        }

        telemetry.update()
    }
}

