package org.firstinspires.ftc.teamcode.pedroPathing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Configurable
@TeleOp
class Example : OpMode() {
    private lateinit var follower: Follower
    private var automatedDrive = false
    private lateinit var pathChain: () -> PathChain
    private lateinit var telemetryM: TelemetryManager
    private var slowMode = false
    private var slowModeMultiplier = 0.5

    companion object {
        /**
         * This is a companion object, which is Kotlin's equivalent of static members.
         * The @JvmField annotation ensures that this variable is accessible as a public static field
         * from Java code, which is useful for setting the starting pose from an Autonomous OpMode.
         * It's nullable (`Pose?`) to handle the case where it's not set.
         */
        @JvmField
        var startingPose: Pose? = null
    }

    override fun init() {
        follower = Constants.createFollower(hardwareMap)

        // The Elvis operator (?:) provides a concise way to handle nulls.
        // If startingPose is not null, it's used; otherwise, a new default Pose is created.
        follower.setStartingPose(startingPose ?: Pose())
        follower.update()

        // Define the path chain using a lambda. Kotlin's syntax for this is very clean.
        // This is a "lazy" generation, meaning the path is only built when the lambda is invoked.
        pathChain = {
            follower.pathBuilder()
                .addPath(Path(BezierLine(follower::getPose, Pose(45.0, 98.0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45.0), 0.8))
                .build()
        }
    }

    override fun start() {
        // The parameter controls whether the Follower should use brake mode on the motors (using it is recommended).
        // To use float mode, add .useBrakeModeInTeleOp(true) to your Drivetrain Constants.
        // If you don't pass anything in, it uses the default (false).
        follower.startTeleopDrive()
    }

    override fun loop() {
        // Call this once per loop
        follower.update()
        telemetryM.update()

        if (!automatedDrive) {
            // Use an 'if' expression to determine the multiplier. This avoids duplicating the
            // follower.setTeleOpDrive() call and makes the code cleaner.
            val multiplier = if (slowMode) slowModeMultiplier else 1.0

            follower.setTeleOpDrive(
                -gamepad1.left_stick_y * multiplier,
                -gamepad1.left_stick_x * multiplier,
                -gamepad1.right_stick_x * multiplier,
                true // Robot Centric (set to false for field-centric)
            )
        }

        // Automated PathFollowing
        if (gamepad1.a) {
            follower.followPath(pathChain()) // Invoke the lambda to get the path
            automatedDrive = true
        }

        // Stop automated following if the follower is done or 'B' is pressed
        if (automatedDrive && (gamepad1.b || !follower.isBusy)) {
            follower.startTeleopDrive()
            automatedDrive = false
        }

        // Toggle Slow Mode
        if (gamepad1.right_bumper) {
            slowMode = !slowMode
        }

        // Optional way to change slow mode strength
        if (gamepad1.x) {
            slowModeMultiplier += 0.25
        }

        // Optional way to change slow mode strength
        if (gamepad1.y) {
            slowModeMultiplier -= 0.25
        }

        telemetryM.debug("position", follower.pose)
        telemetryM.debug("velocity", follower.velocity)
        telemetryM.debug("automatedDrive", automatedDrive)
    }
}
