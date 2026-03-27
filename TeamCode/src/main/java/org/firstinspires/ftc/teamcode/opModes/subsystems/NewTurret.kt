package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goal
import kotlin.math.atan2

object NewTurret : Subsystem {

    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    var trackingActive = false
    var targetPosition = 0.5  // 0.5 = straight backwards

    override fun initialize() {
        turret1 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret1")
        turret2 = ActiveOpMode.hardwareMap.get(Servo::class.java, "turret2")
        turret1.position = 0.5
        turret2.position = 0.5
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
        val x = follower.pose.x
        val y = follower.pose.y
        val heading = follower.heading

        val turretOffsetX = 0.0
        val turretOffsetY = 0.0
        val tx = x + turretOffsetX
        val ty = y + turretOffsetY

        val targetAngle = if (PoseStorage.blueAlliance) {
            Math.PI - atan2(goal.y - ty, goal.x - tx)
        } else {
            atan2(goal.y - ty, goal.x - (144.0 - tx))
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