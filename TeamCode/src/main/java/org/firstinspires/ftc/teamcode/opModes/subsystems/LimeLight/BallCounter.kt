package org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.roundToInt

object BallCounter : Subsystem {
    
    lateinit var limelight: Limelight3A
        private set

    var count: Int = 0
        private set

    /**
     * Pipeline index to use for counting; can be overridden by test opmodes.
     */
    @JvmField
    var countingPipeline = 0

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        
        // Switch to our counting pipeline
        limelight.pipelineSwitch(countingPipeline)
        
        // Start processing
        limelight.start()
    }

    private val updateCountCommand = InstantCommand {
        val result = limelight.latestResult
        
        if (result != null && result.isValid) {
            var estimatedTotal = 0
            
            // Logic for Color Blobs
            for (blob in result.colorResults) {
                val corners = blob.targetCorners
                if (corners != null && corners.size >= 4) {
                    // Extract X and Y coordinates
                    val xCoords = corners.map { it[0] }
                    val yCoords = corners.map { it[1] }
                    
                    val width = xCoords.max() - xCoords.min()
                    val height = yCoords.max() - yCoords.min()
                    
                    if (height > 0) {
                        // Aspect ratio calculation
                        val ratio = width / height
                        
                        // Estimate count based on ratio
                        // 1.0ish = 1 ball, 2.0ish = 2 balls, etc.
                        val ballsInThisBlob = ratio.roundToInt().coerceAtLeast(1)
                        estimatedTotal += ballsInThisBlob
                    }
                } else {
                    // Fallback if no corners: count as 1
                    estimatedTotal += 1
                }
            }
            
            // If no color results, fallback to detector results count
            if (result.colorResults.isEmpty()) {
                estimatedTotal = result.detectorResults.size
            }
            
            count = estimatedTotal
        } else {
            count = 0
        }
    }

    override val defaultCommand = PerpetualCommand(updateCountCommand).requires(this)
}
