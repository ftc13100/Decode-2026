package org.firstinspires.ftc.teamcode.opModes.teleOp.limelight

import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.core.components.SubsystemComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.BallCounter

// @TeleOp(name = "Limelight Ball Counter Test")
class LimelightBallTest : NextFTCOpMode() {

    init {
        addComponents(
            // Register the BallCounter so it initializes and updates automatically
            SubsystemComponent(BallCounter)
        )
    }

    override fun onUpdate() {
        // Display the count on the Driver Station
        telemetry.addData("Status", "Limelight counting active")
        telemetry.addData("Ball Count", BallCounter.count)
        
        // Help debugging: tell us if the target is actually valid
        val result = BallCounter.limelight.latestResult
        telemetry.addData("Limelight Has Target", result?.isValid ?: false)
        
        telemetry.update()
    }
}
