package org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.PerpetualCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.atan2
import kotlin.math.hypot

object GreenCount : Subsystem {

    private const val GREEN_PIPELINE = 7
    private const val DOUBLE_BALL_RATIO_THRESHOLD = 1.4

    lateinit var limelight: Limelight3A
        private set

    var totalBallCount: Int = 0
        private set

    var detectionCount: Int = 0
        private set

    var detectionBreakdown: List<String> = emptyList()
        private set

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(GREEN_PIPELINE)
        limelight.start()
    }

    private val updateCountCommand = InstantCommand {
        val result = limelight.latestResult
        if (result == null) {
            totalBallCount = 0
            detectionCount = 0
            detectionBreakdown = emptyList()
            return@InstantCommand
        }

        val blobs = result.colorResults
        detectionCount = blobs.size

        var total = 0
        val lines = mutableListOf<String>()

        for ((index, blob) in blobs.withIndex()) {
            val ratio = orientedAspectRatio(blob.targetCorners)
            val balls = if (ratio == null || ratio < DOUBLE_BALL_RATIO_THRESHOLD) 1 else 2
            total += balls

            if (ratio == null) {
                lines.add("D${index + 1}: ratio=n/a balls=$balls")
            } else {
                lines.add("D${index + 1}: ratio=${"%.2f".format(ratio)} balls=$balls")
            }
        }

        totalBallCount = total
        detectionBreakdown = lines
    }

    private fun orientedAspectRatio(corners: List<List<Double>>?): Double? {
        if (corners == null || corners.size < 4) return null

        val points = corners.take(4).map { it[0] to it[1] }
        val centerX = points.map { it.first }.average()
        val centerY = points.map { it.second }.average()

        val ordered = points.sortedBy { atan2(it.second - centerY, it.first - centerX) }
        if (ordered.size < 4) return null

        val e0 = distance(ordered[0], ordered[1])
        val e1 = distance(ordered[1], ordered[2])
        val e2 = distance(ordered[2], ordered[3])
        val e3 = distance(ordered[3], ordered[0])

        val sideA = (e0 + e2) / 2.0
        val sideB = (e1 + e3) / 2.0

        val shortSide = minOf(sideA, sideB)
        val longSide = maxOf(sideA, sideB)
        if (shortSide <= 1e-6) return null

        return longSide / shortSide
    }

    private fun distance(a: Pair<Double, Double>, b: Pair<Double, Double>): Double {
        return hypot(a.first - b.first, a.second - b.second)
    }

    override val defaultCommand = PerpetualCommand(updateCountCommand).requires(this)
}


