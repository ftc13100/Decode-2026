package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage

object AutoPark : Subsystem {

    val currentPose
        get() = PedroComponent.follower.pose

    val parkPose
        get() = if (PoseStorage.blueAlliance) {
            Pose(131.3, 36.0, Math.toRadians(0.0)).mirror()
        } else {
            Pose(131.3, 36.0, Math.toRadians(0.0))
        }


    lateinit var liftPath: PathChain

    fun buildPaths() {

        liftPath = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(currentPose, parkPose))
            .setLinearHeadingInterpolation(currentPose.heading, parkPose.heading)
            .build()

    }
}