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
            Pose(123.000, 123.000),

            Pose(84.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(128.0), Math.toRadians(128.0))

        .build()

    val Path2 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(84.000, 84.000),

            Pose(123.513, 83.741)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path5 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(123.513, 83.741),

            Pose(128.515, 73.401)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(93.0))

        .build()

    val Path3 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(128.515, 73.401),

            Pose(84.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(93.0), Math.toRadians(-59.0))
        .setReversed()
        .build()

    val Path4 = follower.pathBuilder().addPath(
        BezierCurve(
            Pose(84.000, 84.000),
            Pose(101.219, 55.014),
            Pose(121.954, 58.595)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path6 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(121.954, 58.595),

            Pose(84.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(-90.0), Math.toRadians(-76.0))
        .setReversed()
        .build()

    val Path7 = follower.pathBuilder().addPath(
        BezierCurve(
            Pose(84.000, 84.000),
            Pose(97.622, 28.216),
            Pose(125.650, 35.151)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path8 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(125.650, 35.151),

            Pose(84.000, 84.000)
        )
    ).setTangentHeadingInterpolation()
        .setReversed()
        .build()

    val Path9 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(84.000, 84.000),

            Pose(120.000, 72.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(-46.0), Math.toRadians(0.0))

        .build()
}
