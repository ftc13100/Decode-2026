package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

object redAutoPaths : Subsystem {
val Path1 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(110.167, 134.621),

        Pose(84.000, 84.000)
    )
).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(0.0))

.build()

val Path2 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(84.000, 84.000),

        Pose(120.000, 84.049)
    )
).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))

.build()

val Path3 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(120.000, 84.049),

        Pose(128.000, 74.800)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path4 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(128.000, 74.800),

        Pose(84.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path5 = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(84.000, 84.000),
        Pose(105.000, 57.000),
        Pose(120.000, 59.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path6 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(120.000, 59.000),

        Pose(84.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path7 = follower.pathBuilder().addPath(
    BezierCurve(
        Pose(84.000, 84.000),
        Pose(88.000, 30.000),
        Pose(120.000, 35.872)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path8 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(120.000, 35.872),

        Pose(84.000, 84.000)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()

val Path9 = follower.pathBuilder().addPath(
    BezierLine(
        Pose(84.000, 84.000),

        Pose(122.704, 73.414)
    )
).setConstantHeadingInterpolation(Math.toRadians(0.0))

.build()
    }







