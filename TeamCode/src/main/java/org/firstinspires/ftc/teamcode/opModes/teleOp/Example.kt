package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Configurable
@TeleOp
class Example : OpMode() {
    private lateinit var follower: Follower
    private var automatedDrive = false
    private lateinit var pathChain: () -> PathChain
    private var slowMode = false
    private var slowModeMultiplier = 0.5

    // Variables to track previous gamepad button states for edge detection (wasPressed behavior)
    private var previousGamepad1A = false
    private var previousGamepad1B = false
    private var previousGamepad1RightBumper = false
    private var previousGamepad1X = false
    private var previousGamepad1Y = false

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

        // --- Gamepad logic with edge detection to simulate "wasPressed" ---

        // Automated PathFollowing
        val currentGamepad1A = gamepad1.a
        if (currentGamepad1A && !previousGamepad1A) {
            follower.followPath(pathChain()) // Invoke the lambda to get the path
            automatedDrive = true
        }
        previousGamepad1A = currentGamepad1A

        // Stop automated following if the follower is done or 'B' is pressed
        val currentGamepad1B = gamepad1.b
        if (automatedDrive && ((currentGamepad1B && !previousGamepad1B) || !follower.isBusy)) {
            follower.startTeleopDrive()
            automatedDrive = false
        }
        previousGamepad1B = currentGamepad1B

        // Toggle Slow Mode
        val currentGamepad1RightBumper = gamepad1.right_bumper
        if (currentGamepad1RightBumper && !previousGamepad1RightBumper) {
            slowMode = !slowMode
        }
        previousGamepad1RightBumper = currentGamepad1RightBumper

        // Optional way to change slow mode strength
        val currentGamepad1X = gamepad1.x
        if (currentGamepad1X && !previousGamepad1X) {
            slowModeMultiplier += 0.25
        }
        previousGamepad1X = currentGamepad1X

        // Optional way to change slow mode strength
        val currentGamepad1Y = gamepad1.y
        if (currentGamepad1Y && !previousGamepad1Y) {
            slowModeMultiplier -= 0.25
        }
        previousGamepad1Y = currentGamepad1Y


        // Standard FTC Telemetry to show key information on the Driver Station.
        telemetry.addData("Pose", "X: %.2f, Y: %.2f, Heading: %.2f", follower.pose.x, follower.pose.y, Math.toDegrees(follower.pose.heading))
        telemetry.addData("Automated Drive Active", automatedDrive)
        telemetry.addData("Slow Mode Active", slowMode)
        telemetry.update()
    }
}

