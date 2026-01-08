package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

object PoseStorage : Subsystem {
    var poseEnd: Pose = Pose()
    var blueAlliance: Boolean = false
    var redAlliance: Boolean = false

    fun endPose() {
        follower.pose.y
        follower.pose.x
        follower.pose.heading

    }
}

