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
    val PGPcontrol = Pose(41.17, 61.82)
    val PGPback = Pose(60.0, 84.0, Math.toRadians(-155.0))

    val gate = Pose(16.0, 66.0, Math.toRadians(180.0))
    val eat = Pose(11.0, 51.64, Math.toRadians(135.0))
    val gateBack = Pose(60.0, 84.0, Math.toRadians(-147.0))

    val PPG = Pose(22.0, 84.0, Math.toRadians(180.0))
    val leave = Pose(26.0,70.0, Math.toRadians(180.0))




    lateinit var startShoot : PathChain
    lateinit var shootPGP: PathChain
    lateinit var PGPshoot: PathChain
    lateinit var shootGate: PathChain
    lateinit var shootPPG: PathChain

    lateinit var gateEat: PathChain
    lateinit var eatShoot: PathChain
    lateinit var goLeave: PathChain

    lateinit var PPGshoot: PathChain


    fun buildPaths() {

        startShoot= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(start, shoot))
            .setLinearHeadingInterpolation(start.heading, shoot.heading)
            .build()
        shootPGP= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(shoot, PGPcontrol, PGP))
            .setTangentHeadingInterpolation()
            .build()
        PGPshoot= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(PGP, PGPback))
            .setLinearHeadingInterpolation(PGP.heading, PGPback.heading)
            .build()
        shootGate= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(PGPback, gate))
            .setLinearHeadingInterpolation(PGPback.heading, gate.heading)
            .build()
        gateEat= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(gate, eat))
            .setLinearHeadingInterpolation(gate.heading, eat.heading)
            .build()
        eatShoot= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(gate, gateBack))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
        shootPPG= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(gateBack, PPG))
            .setLinearHeadingInterpolation(gateBack.heading, PPG.heading)
            .build()
        PPGshoot= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(PPG, gateBack))
            .setLinearHeadingInterpolation(PPG.heading, gateBack.heading)
            .build()
        goLeave= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(gateBack, leave))
            .setLinearHeadingInterpolation(gateBack.heading, leave.heading)
            .build()
    }


    }