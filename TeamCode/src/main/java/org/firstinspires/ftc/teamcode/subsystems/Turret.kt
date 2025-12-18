package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import kotlin.math.abs
import kotlin.math.atan2

@Configurable
object Turret : Subsystem {
    @JvmField var target = 0.0
    @JvmField var turretActive = false
    @JvmField var goalTrackingActive = false
    @JvmField var turretReady = false
    @JvmField var turretReadyMs = 0.0
    @JvmField var startPosition = 0.0
    @JvmField var leftLimit = 0.0
    @JvmField var rightLimit = 0.0
    @JvmField var targetAngle = 0.0
    @JvmField var turretAngle = 0.0
    @JvmField var heading = 0.0
    @JvmField var turretError = 0.0
    @JvmField var turretTolearanceCount = 0
    @JvmField var posPIDCoefficients = PIDCoefficients(0.0095, 0.0, 0.0001)
    val turret = MotorEx("turret").brakeMode()
    private val runtime = ElapsedTime()

    val controlSystem = controlSystem {
        posPid(posPIDCoefficients)
    }

    fun initPos()
    {
        startPosition = turret.currentPosition
        target = startPosition
        rightLimit = startPosition + 850.0
        leftLimit = startPosition - 850.0
        turn(0.0)
    }
    /**
     * PID-Only turn: updates target and turret logic handles the rest.
     */
    fun turn(posAdj: Double) {
        target = (target + posAdj).coerceIn(leftLimit, rightLimit)

        goalTrackingActive = false       // stop tracking if active
        turretActive = true              // PID hold mode ON
        turretReady = false
        turretTolearanceCount = 0
        runtime.reset()

        // Ready automatically when close enough (handled in periodic)
    }

    fun resetToStartPosition() {
        turn(startPosition - turret.currentPosition)
    }
    /**
     * Enable auto tracking; PID will follow continuously in periodic.
     */
    fun trackTarget() {
        goalTrackingActive = true
        turretActive = false
        turretReady = false
        turretReadyMs = 0.0
    }

    /**
     * Compute a new turret angle target during auto-tracking.
     */
    fun updateTarget() {
        val goal = Pose(16.0, 132.0)
        val x = abs(follower.pose.x)
        val y = abs(follower.pose.y)
        heading = follower.heading

        targetAngle = if (PoseStorage.blueAlliance) {
            Math.PI - atan2(abs(goal.y - y), abs(goal.x - x))
        } else {
            atan2(abs(goal.y - y), abs(goal.x - (144 - x)))
        }

        turretAngle = heading -
                (2 * Math.PI * (turret.currentPosition - startPosition) / (537.7 * 6.0))

        turretError = targetAngle - turretAngle
        if (turretError > Math.PI) turretError -= 2 * Math.PI

        if(goalTrackingActive) {
            target = (turret.currentPosition - (turretError / (2 * Math.PI) * (537.7 * 6.0))).coerceIn(
                leftLimit,
                rightLimit
            )
        }
    }

    override fun periodic() {
        val current = turret.currentPosition
        updateTarget()

        // 1. TARGET TRACKING MODE
        if (goalTrackingActive) {
            controlSystem.goal = KineticState(target)
            turret.power = controlSystem.calculate(turret.state)
            return
        }

        // 2. PID HOLD MODE
        if (turretActive) {
            controlSystem.goal = KineticState(position = target)
            turret.power = controlSystem.calculate(turret.state)

            // Determine when turret is "ready"
            if (!turretReady && abs(current - target) < 5.0 && ++turretTolearanceCount >= 5) {
                turretReady = true
                turretReadyMs = runtime.milliseconds()
            }
            return
        }

        // 3. IDLE MODE
        turret.power = 0.0
    }
}
