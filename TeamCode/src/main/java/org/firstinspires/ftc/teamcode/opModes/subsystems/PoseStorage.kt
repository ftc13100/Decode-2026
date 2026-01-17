package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.Subsystem

object PoseStorage : Subsystem {
    var poseEnd = Pose()
    var blueAlliance = false
    var redAlliance = false

    var turretValid = false
        private set

    var turretStartPos = -1.0
        set(value) {
            turretValid = true
            field = value
        }
}

