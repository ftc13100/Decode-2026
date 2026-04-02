package org.firstinspires.ftc.teamcode.opModes.teleOp.limelight

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.PurpleCount

@TeleOp(name = "purplecount")
class PurpleCountOpMode : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(PurpleCount)
        )
    }

    override fun onUpdate() {
        val result = PurpleCount.limelight.latestResult

        telemetry.addData("Status", "Purple counter active")
        telemetry.addData("Pipeline", 8)
        telemetry.addData("Detections", PurpleCount.detectionCount)
        telemetry.addData("Final Total", PurpleCount.totalBallCount)
        telemetry.addData("Limelight Has Target", result?.isValid ?: false)

        for (line in PurpleCount.detectionBreakdown.take(10)) {
            telemetry.addLine(line)
        }

        telemetry.update()
    }
}

