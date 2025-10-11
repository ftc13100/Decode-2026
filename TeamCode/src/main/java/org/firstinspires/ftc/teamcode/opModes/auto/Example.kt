package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@Autonomous(name = "Example Auto", group = "Examples")
class Example : OpMode() {
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private var pathState: Int = 0

    private lateinit var scorePreload: Path
    private lateinit var grabPickup1: PathChain
    private lateinit var scorePickup1: PathChain
    private lateinit var grabPickup2: PathChain
    private lateinit var scorePickup2: PathChain
    private lateinit var grabPickup3: PathChain
    private lateinit var scorePickup3: PathChain

    // Fixed poses from your Java code
    private val startPose = Pose(28.5, 128.0, Math.toRadians(180.0))
    private val scorePose = Pose(60.0, 85.0, Math.toRadians(135.0))
    private val pickup1Pose = Pose(37.0, 121.0, Math.toRadians(0.0))
    private val pickup2Pose = Pose(43.0, 130.0, Math.toRadians(0.0))
    private val pickup3Pose = Pose(49.0, 135.0, Math.toRadians(0.0))

    /** Called once when the OpMode is initialized (before "Init" is pressed) **/
    override fun init() {
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        // Replace Constants.createFollower(hardwareMap) with however you normally create your follower
        follower = Constants.createFollower(hardwareMap)

        buildPaths()
        follower.setStartingPose(startPose)
    }

    /** Runs continuously after Init is pressed but before Play **/
 /*   override fun init_loop() {
        // Nothing special for now
    }
*/
    /** Called once when "Play" is pressed **/
    override fun start() {
        opmodeTimer.resetTimer()
        setPathState(0)
    }

    /** Main loop, runs repeatedly while OpMode is active **/
    override fun loop() {
        // These must be called continuously
        follower.update()
        autonomousPathUpdate()

        // Debug telemetry
        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower.pose.x)
        telemetry.addData("y", follower.pose.y)
        telemetry.addData("heading", follower.pose.heading)
        telemetry.update()
    }

    /** Called once when OpMode ends **/
    override fun stop() {
        // Nothing required, follower should stop automatically
    }

    private fun buildPaths() {
        scorePreload = Path(BezierLine(startPose, scorePose))
        scorePreload.setLinearHeadingInterpolation(startPose.heading, scorePose.heading)

        grabPickup1 = follower.pathBuilder()
            .addPath(BezierLine(scorePose, pickup1Pose))
            .setLinearHeadingInterpolation(scorePose.heading, pickup1Pose.heading)
            .build()

        scorePickup1 = follower.pathBuilder()
            .addPath(BezierLine(pickup1Pose, scorePose))
            .setLinearHeadingInterpolation(pickup1Pose.heading, scorePose.heading)
            .build()

        grabPickup2 = follower.pathBuilder()
            .addPath(BezierLine(scorePose, pickup2Pose))
            .setLinearHeadingInterpolation(scorePose.heading, pickup2Pose.heading)
            .build()

        scorePickup2 = follower.pathBuilder()
            .addPath(BezierLine(pickup2Pose, scorePose))
            .setLinearHeadingInterpolation(pickup2Pose.heading, scorePose.heading)
            .build()

        grabPickup3 = follower.pathBuilder()
            .addPath(BezierLine(scorePose, pickup3Pose))
            .setLinearHeadingInterpolation(scorePose.heading, pickup3Pose.heading)
            .build()

        scorePickup3 = follower.pathBuilder()
            .addPath(BezierLine(pickup3Pose, scorePose))
            .setLinearHeadingInterpolation(pickup3Pose.heading, scorePose.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                setPathState(1)
            }
            1 -> if (!follower.isBusy()) {
                follower.followPath(grabPickup1, true)
                setPathState(2)
            }
            2 -> if (!follower.isBusy()) {
                follower.followPath(scorePickup1, true)
                setPathState(3)
            }
            3 -> if (!follower.isBusy()) {
                follower.followPath(grabPickup2, true)
                setPathState(4)
            }
            4 -> if (!follower.isBusy()) {
                follower.followPath(scorePickup2, true)
                setPathState(5)
            }
            5 -> if (!follower.isBusy()) {
                follower.followPath(grabPickup3, true)
                setPathState(6)
            }
            6 -> if (!follower.isBusy()) {
                follower.followPath(scorePickup3, true)
                setPathState(7)
            }
            7 -> if (!follower.isBusy()) {
                setPathState(-1) // finished
            }
        }
    }

    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
    }
}
