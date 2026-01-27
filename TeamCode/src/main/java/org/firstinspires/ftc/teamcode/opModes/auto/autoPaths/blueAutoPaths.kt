package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

object blueAutoPaths : Subsystem {

    val Path1 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(21.000, 123.000),

            Pose(60.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(52.0), Math.toRadians(52.0))

        .build()

    val Path2 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(60.000, 84.000),

            Pose(20.487, 83.741)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path5 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(20.487, 83.741),

            Pose(15.485, 73.401)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(87.0))

        .build()

    val Path3 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(15.485, 73.401),

            Pose(60.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(87.0), Math.toRadians(239.0))
        .setReversed()
        .build()

    val Path4 = follower.pathBuilder().addPath(
        BezierCurve(
            Pose(60.000, 84.000),
            Pose(42.781, 55.014),
            Pose(22.046, 58.595)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path6 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(22.046, 58.595),

            Pose(60.000, 84.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(256.0))
        .setReversed()
        .build()

    val Path7 = follower.pathBuilder().addPath(
        BezierCurve(
            Pose(60.000, 84.000),
            Pose(46.378, 28.216),
            Pose(18.350, 35.151)
        )
    ).setTangentHeadingInterpolation()

        .build()

    val Path8 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(18.350, 35.151),

            Pose(60.000, 84.000)
        )
    ).setTangentHeadingInterpolation()
        .setReversed()
        .build()

    val Path9 = follower.pathBuilder().addPath(
        BezierLine(
            Pose(60.000, 84.000),

            Pose(24.000, 72.000)
        )
    ).setLinearHeadingInterpolation(Math.toRadians(226.0), Math.toRadians(180.0))

        .build()
}
