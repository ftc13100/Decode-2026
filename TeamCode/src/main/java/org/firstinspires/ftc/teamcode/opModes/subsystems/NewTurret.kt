package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goal
import kotlin.math.abs
import kotlin.math.atan2

@Configurable
object NewTurret : Subsystem {

    private lateinit var turret1: Servo
    private lateinit var turret2: Servo

    var targetPosition = 0.5  // 0.5 = straight back

    @JvmField
    var goalTrackingActive = false

    var leftLimit = 0.0   // Servo min
    var rightLimit = 1.0  // Servo max

    private const val SERVO_MAX_DEG = 300.0 // Servo full travel in degrees

    override fun initialize() {
        turret1 = dev.nextftc.ftc.ActiveOpMode.hardwareMap.get(Servo::class.java, "turret1")
        turret2 = dev.nextftc.ftc.ActiveOpMode.hardwareMap.get(Servo::class.java, "turret2")
        turret1.position = targetPosition
        turret2.position = targetPosition
    }

    fun trackTarget() {
        goalTrackingActive = true
    }

    fun stopTracking() {
        goalTrackingActive = false
    }

    fun toAngle(angle: Double) =
        InstantCommand {
            turret1.position = angle
            turret2.position = angle
        }

    private fun updateTarget() {

        if (!goalTrackingActive) return

        val x = abs(follower.pose.x)
        val y = abs(follower.pose.y)

        // Compute target angle in degrees
        val targetAngleDeg = if (PoseStorage.blueAlliance) {
            180.0 - Math.toDegrees(atan2(abs(goal.y - y), abs(goal.x - x)))
        } else {
            Math.toDegrees(atan2(abs(goal.y - y), abs(goal.x - (144.0 - x))))
        }
//
//        // Map angle to servo position
//        val servoPos = (targetAngleDeg / SERVO_MAX_DEG).coerceIn(leftLimit, rightLimit)
//        targetPosition = servoPos
    }

    override fun periodic() {
//        updateTarget()
//        turret1.position = targetPosition
//        turret2.position = targetPosition
    }
}
