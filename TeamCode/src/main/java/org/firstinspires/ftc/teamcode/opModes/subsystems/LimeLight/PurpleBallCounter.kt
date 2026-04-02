package org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object PurpleBallCounter : Subsystem {

    private const val PURPLE_COUNT_PIPELINE = 8
    private const val SMOOTHING_WINDOW = 5
    private const val EMPTY_FRAME_HOLD = 8
    private const val MAX_COUNT = 6

    // Tune this upward if purple keeps undercounting.
    @JvmField
    var ratioGain = 1.20

    lateinit var limelight: Limelight3A
        private set

    var count: Int = 0
        private set

    var largestBlobRatio: Double = 0.0
        private set

    var adjustedLargestBlobRatio: Double = 0.0
        private set

    var rawCount: Int = 0
        private set

    var validCornerBlobCount: Int = 0
        private set

    var countingMode: String = "NONE"
        private set

    private val recentCounts = ArrayDeque<Int>()
    private var emptyFrameStreak = 0
    private var lastNonEmptyRawCount = 0

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(PURPLE_COUNT_PIPELINE)
        limelight.start()
    }

    private val updateCountCommand = InstantCommand {
        val result = limelight.latestResult

        if (result != null) {
            var estimatedTotal: Int
            var maxRatioSeen = 0.0
            var maxAdjustedRatioSeen = 0.0
            var validBlobCount = 0

            val blobs = result.colorResults
            if (blobs.isEmpty()) {
                emptyFrameStreak += 1
                countingMode = "HOLD"
                estimatedTotal = if (emptyFrameStreak <= EMPTY_FRAME_HOLD) lastNonEmptyRawCount else 0
            } else {
                emptyFrameStreak = 0
                estimatedTotal = blobs.size.coerceAtLeast(1)

                for (blob in blobs) {
                    val corners = blob.targetCorners
                    if (corners != null && corners.size >= 4) {
                        val xCoords = corners.map { it[0] }
                        val yCoords = corners.map { it[1] }

                        val width = xCoords.max() - xCoords.min()
                        val height = yCoords.max() - yCoords.min()

                        if (height > 0) {
                            validBlobCount += 1
                            val ratio = width / height
                            val adjustedRatio = ratio * ratioGain
                            if (ratio > maxRatioSeen) maxRatioSeen = ratio
                            if (adjustedRatio > maxAdjustedRatioSeen) maxAdjustedRatioSeen = adjustedRatio
                        }
                    }
                }

                if (validBlobCount > 0) {
                    val ratioCount = mapRatioToBallCount(maxAdjustedRatioSeen)
                    estimatedTotal = maxOf(estimatedTotal, ratioCount)
                    countingMode = "RATIO+FLOOR"
                } else {
                    countingMode = "BLOB_ONLY"
                }
            }

            estimatedTotal = estimatedTotal.coerceIn(0, MAX_COUNT)
            rawCount = estimatedTotal
            largestBlobRatio = maxRatioSeen
            adjustedLargestBlobRatio = maxAdjustedRatioSeen
            validCornerBlobCount = validBlobCount
            count = smoothCount(estimatedTotal)
            if (estimatedTotal > 0) lastNonEmptyRawCount = estimatedTotal
        } else {
            largestBlobRatio = 0.0
            adjustedLargestBlobRatio = 0.0
            rawCount = 0
            validCornerBlobCount = 0
            countingMode = "NO_RESULT"
            emptyFrameStreak = 0
            lastNonEmptyRawCount = 0
            recentCounts.clear()
            count = 0
        }
    }

    private fun mapRatioToBallCount(adjustedRatio: Double): Int = when {
        adjustedRatio < 1.35 -> 1
        adjustedRatio < 2.15 -> 2
        adjustedRatio < 3.00 -> 3
        adjustedRatio < 3.85 -> 4
        else -> 5
    }

    private fun smoothCount(newCount: Int): Int {
        recentCounts.addLast(newCount)
        if (recentCounts.size > SMOOTHING_WINDOW) recentCounts.removeFirst()

        val sorted = recentCounts.sorted()
        return sorted[sorted.size / 2]
    }

    override val defaultCommand = PerpetualCommand(updateCountCommand).requires(this)
}


