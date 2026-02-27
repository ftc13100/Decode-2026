package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent

object blueAutoPaths : Subsystem {
    val start = Pose(32.325, 135.89, Math.toRadians(180.0))
    val shoot = Pose(60.0, 84.0, Math.toRadians(-131.0))

    val PGP = Pose(22.0, 60.0, Math.toRadians(-174.0))
    val PGPcontrol = Pose(41.1728813559322, 61.82033898305083)
    val PGPback = Pose(60.0, 84.0, Math.toRadians(-155.0))

    val gate = Pose(16.0, 66.0, Math.toRadians(180.0))
    val eat = Pose(11.0, 51.64, Math.toRadians(135.0))
    val gateBack = Pose(60.0, 84.0, Math.toRadians(-147.0))

    val PPG = Pose(22.0, 84.0, Math.toRadians(180.0))
    //uses gateBack for PPG intake

    val leave = Pose(26.0,70.0, Math.toRadians(180.0))




    fun buildPaths() {
    }
    }