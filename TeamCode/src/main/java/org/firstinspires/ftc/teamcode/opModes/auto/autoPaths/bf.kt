
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import SpindexerAuto
import android.graphics.SweepGradient
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.NewTurret
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.Spindexer
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.TurretAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "bf")
class bf: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Shooter, ShooterAngle, Intake, PoseStorage,
                 SpindexerAuto, TurretAuto, NewTurret
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val start = Pose(56.24784853700516, 8.247848537005176, Math.toRadians(180.0))

    val corn = Pose(9.0, 9.0, Math.toRadians(180.0))
    val cornback = Pose(20.0, 9.0, Math.toRadians(180.0))

    val row = Pose(7.845094664371786, 35.25129087779688, Math.toRadians(180.0))

    val rowControl = Pose(47.66523235800346, 37.82530120481927, Math.toRadians(180.0))

    val sweepUp = Pose(10.734939759036147, 45.860585197934604, Math.toRadians(-120.0))

    val sweepDown = Pose(10.858864027538726, 5.22203098106715, Math.toRadians(-120.0))

    lateinit var startCorn : PathChain
    lateinit var path1 : PathChain

    lateinit var path2 : PathChain

    lateinit var cornStart : PathChain

    lateinit var startRow : PathChain
    lateinit var startSweep : PathChain
    lateinit var sweepUpSweepDown : PathChain

    lateinit var rowStart : PathChain
    lateinit var sweepDownStart : PathChain




    fun buildPaths() {

        startCorn = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(start, corn))
            .setLinearHeadingInterpolation(start.heading, corn.heading)
            .setNoDeceleration()
            .build()
        path1 = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(corn, cornback))
            .setLinearHeadingInterpolation(start.heading, corn.heading)
            .setNoDeceleration()
            .build()
        path2 = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(cornback, corn))
            .setLinearHeadingInterpolation(start.heading, corn.heading)
            .setNoDeceleration()
            .build()
        cornStart = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(corn, start))
            .setLinearHeadingInterpolation(corn.heading, start.heading)
            .build()
        startRow = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(start, rowControl, row))
            .setLinearHeadingInterpolation(start.heading, row.heading)
            .setNoDeceleration()
            .build()
        rowStart = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(row, start))
            .setLinearHeadingInterpolation(row.heading, start.heading)
            .build()
        startSweep = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(start, sweepUp))
            .setLinearHeadingInterpolation(start.heading, sweepUp.heading, 0.2)
            .setNoDeceleration()
            .build()
        sweepUpSweepDown = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(sweepUp, sweepDown))
            .setLinearHeadingInterpolation(sweepUp.heading, sweepDown.heading, 0.5)
            .setNoDeceleration()
            .build()
        sweepDownStart = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(sweepDown, start))
            .setLinearHeadingInterpolation(sweepDown.heading, start.heading, 0.5)
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
    }

    val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_mid,
                    Shooter.spinAtSpeed(2000.0),
                ),
                Intake.spinFastAuto,
                SpindexerAuto.toShoot,
                ParallelGroup(
                    SpindexerAuto.toIntake,
                    FollowPath(startCorn),
                ),
                FollowPath(path1),
                FollowPath(path2),
                FollowPath(cornStart),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toIntake,
                   FollowPath(startRow),
                ),
                FollowPath(rowStart),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toIntake,
                    FollowPath(startSweep)),
                FollowPath(sweepUpSweepDown),
                FollowPath(sweepDownStart),
                SpindexerAuto.toShoot,
                ParallelGroup(
                    Intake.spinFastAuto,
                    SpindexerAuto.toIntake,
                    FollowPath(startSweep)),
                FollowPath(sweepUpSweepDown),
                FollowPath(sweepDownStart),
                SpindexerAuto.toShoot,


                )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Spindexer.toIntakePos

    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(start)
        buildPaths()
        PoseStorage.blueAlliance = true
        PoseStorage.redAlliance = false
        NewTurret.goalTrackingActive = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
        telemetry.addData("pos", "%.3f", SpindexerAuto.spindexer.currentPosition);

        telemetry.update()
    }

}
