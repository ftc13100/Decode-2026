package org.firstinspires.ftc.teamcode.opModes.teleOp.limelight

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.PurpleBallCounter

@TeleOp(name = "Purple Ball Counter Test")
class PurpleBallCounterTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(PurpleBallCounter)
        )
    }

    override fun onUpdate() {
        val result = PurpleBallCounter.limelight.latestResult

        telemetry.addData("Status", "Purple counting active")
        telemetry.addData("Pipeline", 8)
        telemetry.addData("Raw Count", PurpleBallCounter.rawCount)
        telemetry.addData("Smoothed Count", PurpleBallCounter.count)
        telemetry.addData("Raw Ratio", "%.2f", PurpleBallCounter.largestBlobRatio)
        telemetry.addData("Adjusted Ratio", "%.2f", PurpleBallCounter.adjustedLargestBlobRatio)
        telemetry.addData("Ratio Gain", "%.2f", PurpleBallCounter.ratioGain)
        telemetry.addData("Mode", PurpleBallCounter.countingMode)
        telemetry.addData("Valid Corner Blobs", PurpleBallCounter.validCornerBlobCount)
        telemetry.addData("Limelight Has Target", result?.isValid ?: false)
        telemetry.addData("Color Blob Count", result?.colorResults?.size ?: 0)
        telemetry.update()
    }
}


