package org.firstinspires.ftc.teamcode.opModes.auto.red

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
import org.firstinspires.ftc.teamcode.opModes.auto.autoPaths.redAutoPaths
import org.firstinspires.ftc.teamcode.opModes.subsystems.Gate
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitPatil
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.TurretAuto
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "redBottom3")
class redBottom3 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitPatil, Shooter, ShooterAngle, Intake, Gate, PoseStorage,
                TurretAuto, redAutoPaths
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val bottomStartPose = Pose(56.0, 7.5, Math.toRadians(90.0)).mirror()
    val bottomShootPose = Pose(56.0, 10.5, Math.toRadians(90.0))
    val bottomHPpose = Pose(11.907244983779883, 8.961972846329473, Math.toRadians(179.5)).mirror()
    val bottomPickUppose = Pose(8.78048780487805, 14.299651567944245, Math.toRadians(90.0)).mirror()

    val bottomLeavePoint = Pose(36.49261083743842, 10.5, Math.toRadians(90.0)).mirror()

    lateinit var bottomHP: PathChain
    lateinit var bottomShoot: PathChain
    lateinit var bottomLeave: PathChain
    lateinit var bottomIntake: PathChain
    lateinit var intakeShoot: PathChain

    lateinit var HPshoot: PathChain


    fun buildPaths() {
        bottomShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomShootPose.heading)
            .build()
        bottomHP = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomHPpose))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomHPpose.heading)
            .build()
        HPshoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomHPpose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomHPpose.heading, bottomShootPose.heading)
            .build()
        intakeShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomPickUppose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomPickUppose.heading, bottomShootPose.heading)
            .build()
        bottomIntake = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomPickUppose))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomPickUppose.heading)
            .build()
        bottomLeave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomLeavePoint))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomLeavePoint.heading)
            .build()
        bottomShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomShootPose.heading)
            .build()
        bottomHP = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomHPpose))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomHPpose.heading)
            .build()
        HPshoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomHPpose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomHPpose.heading, bottomShootPose.heading)
            .build()
        intakeShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomPickUppose, bottomShootPose))
            .setLinearHeadingInterpolation(bottomPickUppose.heading, bottomShootPose.heading)
            .build()
        bottomIntake = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomPickUppose))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomPickUppose.heading)
            .build()
        bottomLeave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomShootPose, bottomLeavePoint))
            .setLinearHeadingInterpolation(bottomShootPose.heading, bottomLeavePoint.heading)
            .build()
    }


        val autoRoutine: Command
        get() =
            SequentialGroup(
                ParallelGroup(
                    ShooterAngle.angle_up,
                    Shooter.spinAtSpeed(1525.0),
                    TurretAuto.toRightMohit,
                    Gate.gate_open,
                    FollowPath(bottomShoot)

                ),
                Intake.spinFastAuto,
                Delay(2.3.seconds),
                ParallelGroup(
                    TurretAuto.toMid,
                    FollowPath(bottomLeave),
                    Gate.gate_close,
                    Intake.spinStop,

                    )

            )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)
        Gate.gate_close()
    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(bottomStartPose)
        redAutoPaths.buildPaths()
        PoseStorage.blueAlliance = false
        PoseStorage.redAlliance = true
        autoRoutine()
    }

    override fun onStop() {
        PoseStorage.poseEnd = PedroComponent.Companion.follower.pose
    }

    override fun onUpdate() {
    }
}