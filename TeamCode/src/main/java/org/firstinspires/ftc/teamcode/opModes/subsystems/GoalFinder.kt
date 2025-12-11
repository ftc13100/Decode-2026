package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.compareTo
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt
import kotlin.text.compareTo
import kotlin.unaryMinus

object GoalFinder : Subsystem {
    var gfActive = false
    var gfModeLL = true
    var gfDone  = false
    var gfDoneMs = 0.0
    
    var gfLLFound = false
    var gfLLLostCount = 0
    var gfStartHeading = 0.0
    var gfLastHeading = 0.0
    var gfTurnedAngle = 0.0
    var gfTurnClockwise = false
    var gfTargetAngle = 0.0
    var gfHeadingError = 0.0
    var gfLLTx = 99.0
    var gfLLTy = 99.0
    var gfLLTa = 99.0
    var gfGoalDistance = 1000.0
    var gfBelowToleranceCount = 0
    private val ALIGNMENT_POWER_COARSE = 0.6
    private val ALIGNMENT_POWER_FINE = 0.2
    private val HEADING_TOLERANCE_COARSE = Math.toRadians(12.0)
    private val HEADING_TOLERANCE_FINE = Math.toRadians(1.0)
    private val LL_TOLERANCE = 1.0
    private val TOLERANCE_TARGET = 12
    private val LL_LOST_MAX = 100
    private val goal = Pose(16.0, 132.0)
    private val shooterToGoalZSqrd = Math.pow(46.0 - 13.5, 2.0)

    private val gfRuntime = ElapsedTime()

    fun normalizeAngle(angle: Double): Double {
        var a = angle
        while (a > Math.PI)  a -= 2 * Math.PI
        while (a < -Math.PI) a += 2 * Math.PI
        return a
    }

    fun findGoal(pose: Pose, heading: Double, llResult: LLResult?, blueAlliance: Boolean) {
        gfActive = true
        gfModeLL = true
        gfBelowToleranceCount = 0
        gfDone = false
        gfDoneMs = 0.0
        gfRuntime.reset()

        gfLLTx = 99.0
        gfLLTy = 99.0
        gfLLTa = 99.0
        gfLLFound = false
        gfLLLostCount = 0
        if (llResult != null && llResult.isValid) {
            gfLLTx = llResult.tx
            gfLLTy = llResult.ty
            gfLLTa = llResult.ta
            gfLLFound = true
        }

        if(abs(gfLLTx) < LL_TOLERANCE) {
            goalFindSuccess()
            return
        }

        if(pose.x > 0.0 && pose.x < 144.0 && pose.y > 0.0 && pose.y < 144.0) {
            val adjX = if (blueAlliance) {
                pose.x
            } else {
                144.0 - pose.x
            }
            gfGoalDistance = Math.sqrt(Math.pow(adjX - goal.x, 2.0) + Math.pow(pose.y - goal.y, 2.0) + shooterToGoalZSqrd)

            gfTargetAngle = if (blueAlliance) {
                Math.PI - atan2(abs(goal.y - pose.y), abs(goal.x - adjX))
            } else {
                atan2(abs(goal.y - pose.y), abs(goal.x - adjX))
            }
            gfHeadingError = normalizeAngle(gfTargetAngle - heading)

            // Initialize turn direction based on heading error calculated from X and Y.
            // Since we are seeing issues with these, after starting the trun, only stop
            // based on April Tag detection or if full 360 turn has completed
            gfTurnClockwise = if(gfHeadingError < 0) true else false
        } else {
            gfGoalDistance = 1000.0
            gfTurnClockwise = true
        }
        gfStartHeading = heading
        gfLastHeading = gfStartHeading
        gfTurnedAngle = 0.0
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

    fun stop()
    {
        gfActive = false
        gfDone = false
    }
    fun calculate(pose: Pose, heading: Double, llResult: LLResult?, blueAlliance: Boolean): Double {
        val adjX = if(blueAlliance) {
            pose.x
        } else {
            144.0 - pose.x
        }

        if(pose.x > 0.0 && pose.x < 144.0 && pose.y > 0.0 && pose.y < 144.0) {
            gfGoalDistance = Math.sqrt(Math.pow(adjX - goal.x, 2.0) + Math.pow(pose.y - goal.y, 2.0) + shooterToGoalZSqrd)
            gfTargetAngle = if (blueAlliance) {
                Math.PI - atan2(abs(goal.y - pose.y), abs(goal.x - adjX))
            } else {
                atan2(abs(goal.y - pose.y), abs(goal.x - adjX))
            }
            gfHeadingError = normalizeAngle(gfTargetAngle - heading)
        }

        if (llResult != null && llResult.isValid) {
            gfLLTx = llResult.tx
            gfLLTy = llResult.ty
            gfLLTa = llResult.ta
            gfLLFound = true
        }

        if(!gfActive)
            return 0.0

        var delta = normalizeAngle(heading - gfLastHeading)
        gfTurnedAngle += delta;
        gfLastHeading = heading;

        if(gfLLFound && !(llResult != null && llResult.isValid)) {
            if(++gfLLLostCount > LL_LOST_MAX) {
                gfModeLL = false
                gfBelowToleranceCount = 0
            }
        }

        if(gfModeLL) {
            if (!gfLLFound) {
                // Check how far have we turned before finding LL. If we have already made one full turn,
                // then stop and point by computed heading angles using odometry provided x, y, heading

                if (abs(gfTurnedAngle) > 2 * Math.PI * 1.05) {
                    gfModeLL = false
                    gfBelowToleranceCount = 0
                } else {
                    return if (gfTurnClockwise) {
                        ALIGNMENT_POWER_COARSE
                    } else {
                        -ALIGNMENT_POWER_COARSE
                    }
                }

            } else {
                if(abs(gfLLTx) < LL_TOLERANCE) {
                    if(++gfBelowToleranceCount >= TOLERANCE_TARGET) {
                        goalFindSuccess()
                        return 0.0
                    }

                    return if((gfLLTx > 0.0)) {
                        ALIGNMENT_POWER_FINE
                    } else {
                        -ALIGNMENT_POWER_FINE
                    }
                }
            }
        }

        // Use computed targetAngle and current heading to find goal
        if(pose.x < 0.0 || pose.x > 144.0 || pose.y < 0.0 || pose.y > 144.0)
        {
            goalFindFailed()
            return 0.0
        }

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
        } else if (++gfBelowToleranceCount < TOLERANCE_TARGET) {
            return if (gfHeadingError > 0) {
                -ALIGNMENT_POWER_FINE
            } else {
                ALIGNMENT_POWER_FINE
            }
        } else {
            goalFindSuccess()
            return 0.0
        }
    }
}
