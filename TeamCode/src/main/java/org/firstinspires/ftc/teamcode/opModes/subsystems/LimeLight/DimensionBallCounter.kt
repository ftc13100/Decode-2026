package org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.roundToInt

/**
 * Ball counter using dimension-based detection.
 * 
 * - One ball appears as a box of known dimensions
 * - Multiple touching balls form a rectangle
 * - Count = rectangle_dimension / ball_dimension
 * 
 * Uses two Limelight pipelines:
 * - Pipeline 7: fikergreencount (green balls)
 * - Pipeline 8: fikerpurplecount (purple balls)
 */
object DimensionBallCounter : Subsystem {
    
    lateinit var limelight: Limelight3A
        private set

    // Counts for each color
    var greenCount: Int = 0
        private set

    var purpleCount: Int = 0
        private set

    // Total across both colors
    var totalCount: Int = 0
        private set

    // Telemetry helpers
    var lastGreenWidth: Double = 0.0
        private set
    var lastGreenHeight: Double = 0.0
        private set
    var lastPurpleWidth: Double = 0.0
        private set
    var lastPurpleHeight: Double = 0.0
        private set

    /**
     * Known dimensions of a single ball (in pixels).
     * Adjust these based on your camera calibration and ball size.
     */
    const val BALL_WIDTH = 30.0   // pixels
    const val BALL_HEIGHT = 30.0  // pixels

    // Pipeline indices
    private const val GREEN_PIPELINE = 7
    private const val PURPLE_PIPELINE = 8

    // Track which pipeline is active
    private var currentPipeline = GREEN_PIPELINE

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        
        // Start with green pipeline
        limelight.pipelineSwitch(GREEN_PIPELINE)
        currentPipeline = GREEN_PIPELINE
        
        // Start processing
        limelight.start()
    }

    /**
     * Count balls based on blob dimensions.
     * For each blob, divide width/height by known ball dimensions.
     */
    private fun countBallsFromBlob(width: Double, height: Double): Int {
        if (width <= 0 || height <= 0) return 0
        
        // Estimate count from width and height
        // If blob is wider than tall, assume multiple balls side-by-side
        val countByWidth = (width / BALL_WIDTH).roundToInt().coerceAtLeast(1)
        val countByHeight = (height / BALL_HEIGHT).roundToInt().coerceAtLeast(1)
        
        // Take the maximum to be conservative (multiple balls would expand in width primarily)
        return countByWidth.coerceAtLeast(1)
    }

    /**
     * Process a single pipeline and count balls.
     * Returns pair of (count, blobDimensions)
     */
    private fun processPipelineForCounts(): Pair<Int, Pair<Double, Double>> {
        val result = limelight.latestResult
        var totalBalls = 0
        var maxWidth = 0.0
        var maxHeight = 0.0

        if (result != null && result.isValid && result.colorResults.isNotEmpty()) {
            // Process all color blobs
            for (blob in result.colorResults) {
                val corners = blob.targetCorners
                if (corners != null && corners.size >= 4) {
                    // Extract bounding box from corners
                    val xCoords = corners.map { it[0] }
                    val yCoords = corners.map { it[1] }
                    
                    val width = xCoords.maxOrNull()!! - xCoords.minOrNull()!!
                    val height = yCoords.maxOrNull()!! - yCoords.minOrNull()!!
                    
                    // Track max dimensions for telemetry
                    if (width > maxWidth) maxWidth = width
                    if (height > maxHeight) maxHeight = height
                    
                    // Count balls in this blob
                    val ballsInBlob = countBallsFromBlob(width, height)
                    totalBalls += ballsInBlob
                } else {
                    // If no corners, count as 1 ball
                    totalBalls += 1
                }
            }
        }

        return Pair(totalBalls, Pair(maxWidth, maxHeight))
    }

    private fun updateGreenInternal() {
        limelight.pipelineSwitch(GREEN_PIPELINE)
        currentPipeline = GREEN_PIPELINE
        
        val (count, dimensions) = processPipelineForCounts()
        greenCount = count
        lastGreenWidth = dimensions.first
        lastGreenHeight = dimensions.second
    }

    private fun updatePurpleInternal() {
        limelight.pipelineSwitch(PURPLE_PIPELINE)
        currentPipeline = PURPLE_PIPELINE
        
        val (count, dimensions) = processPipelineForCounts()
        purpleCount = count
        lastPurpleWidth = dimensions.first
        lastPurpleHeight = dimensions.second
    }

    /**
     * Combined update that processes both pipelines and updates total.
     */
    private val updateBothCommand = InstantCommand {
        // Update green
        updateGreenInternal()
        Thread.sleep(10) // Small delay for pipeline switch
        
        // Update purple
        updatePurpleInternal()
        Thread.sleep(10)
        
        // Calculate total
        totalCount = greenCount + purpleCount
    }

    override val defaultCommand = PerpetualCommand(updateBothCommand).requires(this)

    /**
     * Force a count update for a specific color.
     */
    fun updateGreen() {
        updateGreenInternal()
    }

    fun updatePurple() {
        updatePurpleInternal()
    }

    /**
     * Switch pipeline manually if needed.
     */
    fun setPipeline(pipelineIndex: Int) {
        limelight.pipelineSwitch(pipelineIndex)
        currentPipeline = pipelineIndex
    }
}

