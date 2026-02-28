package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret.TURRET_TICKS_TO_RADS
import org.firstinspires.ftc.teamcode.opModes.subsystems.Turret.turretCurrentPos
import kotlin.math.pow
import kotlin.math.sqrt


object NewGoalFinder {
    const val TURRET_OFFSET = 3.055

    // Matrix multiplication
    operator fun SimpleMatrix.times(other: SimpleMatrix): SimpleMatrix =
        this.mult(other)

    // Matrix addition
    operator fun SimpleMatrix.plus(other: SimpleMatrix): SimpleMatrix =
        this.plus(other)

    // Matrix subtraction
    operator fun SimpleMatrix.minus(other: SimpleMatrix): SimpleMatrix =
        this.minus(other)

    // Scalar multiply
    operator fun SimpleMatrix.times(scalar: Double): SimpleMatrix =
        this.scale(scalar)

    // Transpose
    fun SimpleMatrix.T(): SimpleMatrix = this.transpose()

    // Inverse
    fun SimpleMatrix.inv(): SimpleMatrix = this.invert()

    fun se2(x: Double, y: Double, theta: Double): SimpleMatrix {
        val c = cos(theta)
        val s = sin(theta)

        return SimpleMatrix(3, 3).apply {
            setRow(0, 0, c, -s, x)
            setRow(1, 0, s,  c, y)
            setRow(2, 0, 0.0, 0.0, 1.0)
        }
    }

    fun turretOffset(oX: Double): SimpleMatrix {
        return SimpleMatrix(3, 3).apply {
            setRow(0, 0, 1.0, 0.0, oX)
            setRow(1, 0, 0.0, 1.0, 0.0)
            setRow(2, 0, 0.0, 0.0, 1.0)
        }
    }

    fun turretRotation(phi: Double): SimpleMatrix {
        val c = cos(phi)
        val s = sin(phi)

        return SimpleMatrix(3, 3).apply {
            setRow(0, 0, c, -s, 0.0)
            setRow(1, 0, s,  c, 0.0)
            setRow(2, 0, 0.0, 0.0, 1.0)
        }
    }

    fun worldToTurret(
        X: Double,
        Y: Double,
        theta: Double,
        oX: Double,
        phi: Double
    ): SimpleMatrix {

        val T_WR = se2(X, Y, theta) // transform robot coordinates into world frame
        val T_off = turretOffset(oX) // turret offset
        val T_phi = turretRotation(phi) // turret current rotation

        return T_WR * T_off * T_phi
    }

    fun goalInTurretFrame(
        X: Double,
        Y: Double,
        theta: Double,
        oX: Double,
        phi: Double,
        xGoal: Double,
        yGoal: Double
    ): SimpleMatrix {

        val T_WT = worldToTurret(X, Y, theta, oX, phi)

        val goalW = SimpleMatrix(3, 1).apply {
            setColumn(0, 0, xGoal, yGoal, 1.0)
        }

        return T_WT.inv() * goalW
    }

    fun turretAimError(
        pose: Pose,
        phi: Double,
        oX: Double = TURRET_OFFSET,
        goalPose: Pose = if (PoseStorage.blueAlliance) {
            ShooterController.goalBlue
        } else {
            ShooterController.goalRed
        }
    ): Double {
        val goalT = goalInTurretFrame(
            pose.x, pose.y, pose.heading, oX, phi, goalPose.x, goalPose.y
        )

        val xT = goalT.get(0)
        val yT = goalT.get(1)

        return atan2(yT, xT)
    }

    fun turretOffsetDistance(): Double {
        val goalPose = if (PoseStorage.blueAlliance) {
            ShooterController.goalBlue
        } else {
            ShooterController.goalRed
        }

        val goalT = goalInTurretFrame(
            follower.pose.x,
            follower.pose.y,
            follower.heading,
            TURRET_OFFSET,
            turretCurrentPos * TURRET_TICKS_TO_RADS,
            goalPose.x,
            goalPose.y
        )

        val xT = goalT.get(0)
        val yT = goalT.get(1)
        return (sqrt(xT.pow(2.0) + yT.pow(2.0) + ShooterController.SHOOTER_TO_GOAL_Z_SQRD))
    }
}