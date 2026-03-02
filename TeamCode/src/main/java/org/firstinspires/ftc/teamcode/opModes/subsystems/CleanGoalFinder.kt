package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.SHOOTER_TO_GOAL_Z_SQRD
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goalBlue
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goalRed
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

object CleanGoalFinder : Subsystem {
    var gfActive = false
    var gfDone = false
    var gfDoneMs = 0.0

    var gfTargetAngle = 0.0
    var gfHeadingError = 0.0

    val gfReady: Boolean
        get() = if (Turret.goalTrackingActive) abs(Turret.turretErrorTicks) < 5.0 else true

    var gfAnglesValid = false
    var gfGoalDistance = 1000.0
    var gfBelowToleranceCount = 0

    private const val ALIGNMENT_POWER_COARSE = 0.6
    private const val ALIGNMENT_POWER_FINE = 0.2
    private val HEADING_TOLERANCE_COARSE = Math.toRadians(12.0)
    private val HEADING_TOLERANCE_FINE = Math.toRadians(1.0)
    private const val HEADING_TOLERANCE_TARGET = 5

    private val gfRuntime = ElapsedTime()

    fun normalizeAngle(angle: Double): Double {
        var a = angle
        while (a > Math.PI) a -= 2 * Math.PI
        while (a < -Math.PI) a += 2 * Math.PI
        return a
    }

    fun findGoal() {
        gfActive = true
        gfBelowToleranceCount = 0
        gfDone = false
        gfDoneMs = 0.0
        gfRuntime.reset()
    }

    fun goalFindSuccess() {
        gfActive = false
        gfDone = true
        gfDoneMs = gfRuntime.milliseconds()
    }

    fun goalFindFailed() {
        gfActive = false
        gfDone = false
    }

    fun stop() {
        gfActive = false
        gfDone = false

        val posAdj = Turret.turret.currentPosition - Turret.startPosition
        Turret.turn(posAdj)
    }

    fun calculateAngles(pose: Pose, blueAlliance: Boolean) {
        val turretOffset = 3.392

        val turretX = pose.x - turretOffset * cos(pose.heading)
        val turretY = pose.y - turretOffset * sin(pose.heading)

        val targetX = if (blueAlliance) goalBlue.x else goalRed.x
        val targetY = goalBlue.y

        val dx = targetX - turretX
        val dy = targetY - turretY

        gfGoalDistance = sqrt(dx.pow(2.0) + dy.pow(2.0) + SHOOTER_TO_GOAL_Z_SQRD)

        //for robot alignment, use normal center calculation in og goalfinder
//        // Target Angle
//        gfTargetAngle = atan2(dy, dx)
//
//        // Heading Alignment
//        gfHeadingError = normalizeAngle(gfTargetAngle - Turret.turretHeadingWithMargin(pose.heading))
//        gfAnglesValid = true
    }

    fun calculateTurn(pose: Pose, blueAlliance: Boolean): Double {
        if (pose.x < 0.0 || pose.x > 144.0 || pose.y < 0.0 || pose.y > 144.0) {
            gfAnglesValid = false
            return 0.0
        }

        calculateAngles(pose, blueAlliance)

        if (!gfActive)
            return 0.0

        if (gfBelowToleranceCount == 0 && abs(gfHeadingError) < HEADING_TOLERANCE_FINE) {
            goalFindSuccess()
            return 0.0
        } else if (abs(gfHeadingError) > HEADING_TOLERANCE_COARSE) {
            return if (gfHeadingError > 0) {
                -ALIGNMENT_POWER_COARSE
            } else {
                ALIGNMENT_POWER_COARSE
            }
        } else if (abs(gfHeadingError) > HEADING_TOLERANCE_FINE) {
            return if (gfHeadingError > 0) {
                -ALIGNMENT_POWER_FINE
            } else {
                ALIGNMENT_POWER_FINE
            }
        } else if (++gfBelowToleranceCount < HEADING_TOLERANCE_TARGET) {
            return if (gfHeadingError > 0) {
                -ALIGNMENT_POWER_FINE
            } else {
                ALIGNMENT_POWER_FINE
            }
        }
        return 0.0
    }
}
