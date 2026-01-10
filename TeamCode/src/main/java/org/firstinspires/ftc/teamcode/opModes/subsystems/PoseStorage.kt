package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem

object PoseStorage : Subsystem {
    var poseEnd: Pose = Pose()
    var blueAlliance: Boolean = false
    var redAlliance: Boolean = false

    var turretStartPos = Double.NaN
}

