package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

object blueAutoPaths : Subsystem {
val shoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(33.833, 134.621),

        Pose(60.000, 84.000)
    )
).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(180.0))

.build()

val shootPPG = follower.pathBuilder().addPath(
    BezierLine(
        Pose(60.000, 84.000),

        Pose(24.000, 84.049)
    )
).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))

.build()

val PPGgate = follower.pathBuilder().addPath(
    BezierLine(
        Pose(24.000, 84.049),

        Pose(16.000, 74.800)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val gateShoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(16.000, 74.800),

        Pose(60.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val shootPGP = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(60.000, 84.000),
        Pose(39.000, 57.000),
        Pose(24.000, 59.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val PGPshoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(24.000, 59.000),

        Pose(60.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val shootGPP = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(60.000, 84.000),
        Pose(56.000, 30.000),
        Pose(24.000, 35.872)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val GPPshoot = follower.pathBuilder().addPath(
    BezierLine(
        Pose(24.000, 35.872),

        Pose(60.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

val shootLeave = follower.pathBuilder().addPath(
    BezierLine(
        Pose(60.000, 84.000),

        Pose(21.296, 73.414)
    )
).setConstantHeadingInterpolation(Math.toRadians(180.0))

.build()

}
