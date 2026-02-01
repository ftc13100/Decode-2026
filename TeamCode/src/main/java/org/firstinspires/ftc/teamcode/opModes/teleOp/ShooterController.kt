package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import kotlin.math.pow

object ShooterController {
    val goal = Pose(6.0, 138.0)
    const val SHOOTER_TO_GOAL_Z_SQRD = 1056.25 // (46.0 - 13.5).pow(2.0)

    data class ShotParameters(val distance: Double, val velocity: Double, val angle: Double)

    /**
     * Calculates shot parameters based on regression equations.
     */
    fun getShot(inputDistance: Double): ShotParameters {

        val distance = inputDistance.coerceIn(30.0, 170.0)

        // Calculate Velocity
        val rawVelocity = (0.0137043 * distance.pow(2)) +
                (2.55405 * distance) +
                840.32177

        // Calculate Angle
        val rawAngle = (4.92475e-9 * distance.pow(4)) -
                (2.16484e-6 * distance.pow(3)) +
                (3.62253e-4 * distance.pow(2)) -
                (0.0277294 * distance) +
                1.33322

        return ShotParameters(
            distance = distance,
            //coerced in tested limits
            velocity = rawVelocity.coerceIn(800.0, 1650.0),
            angle = rawAngle.coerceIn(0.5, 0.7)
        )
    }

    fun applyShot(params: ShotParameters) {
        // Set the hood angle
        ShooterAngle.targetPosition = params.angle

        // Schedule the movement and the flywheel spin
        CommandManager.scheduleCommand(
            ShooterAngle.update()
        )

        CommandManager.scheduleCommand(
            Shooter.spinAtSpeed(params.velocity)
        )
    }
}