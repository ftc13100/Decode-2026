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

    var targetServoPosition = 0.5  // 0.5 = straight back
    var targetAngleRobotRef: Double = 180.0 // 0 is facing forward, CCW increasing
    var targetAngleField: Double = 270.0 // 0 is right, increases CCW

    @JvmField
    var goalTrackingActive = false

    var servoLeftLimit = 0.0   // Servo min
    var servoRightLimit = 1.0  // Servo max

    // 60-120 is deadzone
    var turretLeftLimit = 120
    var turretRightLimit = 60

    private const val SERVO_MAX_DEG = 300.0 // Servo full travel in degrees

    override fun initialize() {
        turret1 = dev.nextftc.ftc.ActiveOpMode.hardwareMap.get(Servo::class.java, "turret1")
        turret2 = dev.nextftc.ftc.ActiveOpMode.hardwareMap.get(Servo::class.java, "turret2")
    }

    fun trackTarget() {
        goalTrackingActive = true
    }

    fun stopTracking() {
        goalTrackingActive = false
    }

    val toPos = InstantCommand {
        turret1.position = targetServoPosition
        turret2.position = targetServoPosition
    }

    // input angle (degrees) is robot centric, 0 is front
    // ccw increasing degrees
    fun toAngle(angle: Double) {
        targetAngleRobotRef = angle
        if (targetAngleRobotRef < 0.0) {
            targetAngleRobotRef += 360.0
        } else if (targetAngleRobotRef > 360.0) {
            targetAngleRobotRef -= 360.0
        }
        setServoPos()
    }

    fun incrementAngle(angle: Double) {
        stopTracking()
        targetAngleRobotRef += angle
        setServoPos()
    }

    fun decrementAngle(angle: Double) {
        stopTracking()
        targetAngleRobotRef -= angle
        setServoPos()
    }

    fun setServoPos() {
        if (targetAngleRobotRef < 0) {
            targetAngleRobotRef += 360.0
        }
        if (targetAngleRobotRef > 360.0) {
            targetAngleRobotRef -= 360.0
        }

        if (targetAngleRobotRef < 30.0 || targetAngleRobotRef > 330.0 ) return

        targetServoPosition = (targetAngleRobotRef - 30.0) / 300.0

        toPos()

    }

    override fun periodic() {
        if (!goalTrackingActive) return

        val x = abs(follower.pose.x)
        val y = abs(follower.pose.y)

        var robotHeading = Math.toDegrees(follower.heading)
        if (robotHeading < 0.0) {
            robotHeading += 360.0
        }
        val turretRobotAdj = 360.0 - robotHeading

        // Compute target angle in degrees
        targetAngleField = if (PoseStorage.blueAlliance) {
            180.0 - Math.toDegrees(atan2(abs(goal.y - y), abs(goal.x - x)))
        } else {
            Math.toDegrees(atan2(abs(goal.y - y), abs(goal.x - (144.0 - x))))
        }

        toAngle(targetAngleField + turretRobotAdj)

    }
}
