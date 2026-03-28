package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goal
import kotlin.math.atan2

@Configurable
object NewTurret : Subsystem {

    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    var trackingActive = false

    @JvmField
    var targetPosition = 0.5  // 0.5 = straight backwards

    // Fields for manually controlling turret via telemetry panels
    @JvmField
    var manualX: Double? = null

    @JvmField
    var manualY: Double? = null

    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret1")
        turret2 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret2")
        turret1.position = targetPosition
        turret2.position = targetPosition
    }

    fun trackTarget() {
        trackingActive = true
    }

    fun stopTracking() {
        trackingActive = false
    }

    fun toAngle(angle: Double) =
        InstantCommand {
            turret1.position = angle
            turret2.position = angle
        }

    private fun updateTarget() {
        val x: Double
        val y: Double

        if (manualX != null && manualY != null) {
            // Use manually entered coordinates
            x = manualX!!
            y = manualY!!
        } else {
            // Use follower pose
            val followerPose = follower.pose
            x = followerPose.x
            y = followerPose.y
        }

        val targetAngle = if (PoseStorage.blueAlliance) {
            Math.PI - atan2(goal.y - y, goal.x - x)
        } else {
            atan2(goal.y - y, goal.x - (144.0 - x))
        }

        // Servo position mapping: 0.5 = straight backwards, full range = 300 degrees
        val servoPos = 0.5 + (targetAngle / (300.0 * Math.PI / 180.0))
        targetPosition = servoPos.coerceIn(0.0, 1.0)
    }

    override fun periodic() {
        if (!trackingActive) return
        updateTarget()
        toAngle(targetPosition)()
    }
}