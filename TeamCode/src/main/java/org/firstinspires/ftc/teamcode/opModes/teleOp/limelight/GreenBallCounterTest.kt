package org.firstinspires.ftc.teamcode.opModes.teleOp.limelight

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.GreenBallCounter

@TeleOp(name = "Green Ball Counter Test")
class GreenBallCounterTest : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(GreenBallCounter)
        )
    }

    override fun onUpdate() {
        val result = GreenBallCounter.limelight.latestResult

        telemetry.addData("Status", "Green counting active")
        telemetry.addData("Pipeline", 7)
        telemetry.addData("Raw Count", GreenBallCounter.rawCount)
        telemetry.addData("Smoothed Count", GreenBallCounter.count)
        telemetry.addData("Raw Ratio", "%.2f", GreenBallCounter.largestBlobRatio)
        telemetry.addData("Adjusted Ratio", "%.2f", GreenBallCounter.adjustedLargestBlobRatio)
        telemetry.addData("Ratio Gain", "%.2f", GreenBallCounter.ratioGain)
        telemetry.addData("Mode", GreenBallCounter.countingMode)
        telemetry.addData("Valid Corner Blobs", GreenBallCounter.validCornerBlobCount)
        telemetry.addData("Limelight Has Target", result?.isValid ?: false)
        telemetry.addData("Color Blob Count", result?.colorResults?.size ?: 0)
        telemetry.update()
    }
}

