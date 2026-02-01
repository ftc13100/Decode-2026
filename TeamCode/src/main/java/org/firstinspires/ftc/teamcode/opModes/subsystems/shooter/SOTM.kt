package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.*
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle

object SOTM : Subsystem {

    var active = false
        private set

    var currentDistance = 0.0
        private set
    var currentVelocity = 0.0
        private set
    var currentAngle = 0.0
        private set

    private const val SMOOTHING = 0.15
    private const val TURRET_TOLERANCE_TICKS = 15.0

    fun enable() {
        active = true
    }

    fun disable() {
        active = false
        Shooter.stallShooter()
        Gate.gate_close()
        Intake.spinStop()
    }

    fun onUpdate() {
        if (!active) return

        // Base shot from regression

        val robotPose = follower.pose
        val goalPose = ShooterController.goal

        val dx = goalPose.x - robotPose.x
        val dy = goalPose.y - robotPose.y
        val baseDistance = hypot(dx, dy)

        val baseShot = ShooterController.getShot(baseDistance)
        val v0 = baseShot.velocity
        val alpha = baseShot.angle

        // Robot velocity decomposition (radial + tangential)

        val robotVel = follower.velocity
        val robotSpeed = robotVel.magnitude
        val robotVelAngle = robotVel.theta
        val goalAngle = atan2(dy, dx)

        val theta = robotVelAngle - goalAngle

        val vRadial = -cos(theta) * robotSpeed
        val vTangential = sin(theta) * robotSpeed

         // Time of flight (horizontal motion)

        val vxBall = v0 * cos(alpha)
        val timeOfFlight = baseDistance / vxBall

         //  Compensated X velocity

        val vxCompensated = baseDistance / timeOfFlight + vRadial
        val vxNew = hypot(vxCompensated, vTangential)


        // New launch angle

        val vy = v0 * sin(alpha)
        val compensatedAngle = atan2(vy, vxNew)
            .coerceIn(0.5, 0.7)

         // New effective distance and velocity

        val compensatedDistance = vxNew * timeOfFlight
        val compensatedShot = ShooterController.getShot(compensatedDistance)

        // smoothing

        currentDistance = compensatedShot.distance
        currentVelocity += (compensatedShot.velocity - currentVelocity) * SMOOTHING
        currentAngle += (compensatedAngle - currentAngle) * SMOOTHING

        Shooter.target = currentVelocity
        ShooterAngle.targetPosition = currentAngle

         // Turret lead compensation

        val turretOffset = atan2(vTangential, vxCompensated)
        Turret.setLeadCompensation(turretOffset)

         // Shoot control

        if (Shooter.shooterReady && GoalFinder.gfReady) {
            Gate.gate_open()
            Intake.spinShoot()
        }
    }
}
