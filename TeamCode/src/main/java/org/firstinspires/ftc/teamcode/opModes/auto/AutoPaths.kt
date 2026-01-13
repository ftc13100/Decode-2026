package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.SHOOTER_TO_GOAL_Z_SQRD
import org.firstinspires.ftc.teamcode.opModes.teleOp.ShooterController.goal
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

object AutoPaths : Subsystem {
    ////////KEY//////////
    //pickUp_____1 == Getting to pick up motif row
    //pickUp_____2 == Ramming into the motif
    //___toShot    == Goes to the shootPose after picking up
    //_____Control == Dilates the straight line into a curve

    ///////DOCS//////////
    //Parallel Groups    == Everything within happens at the same time
    //Sequential Groups  == Everything within happens chronologically
    private val startPose = Pose(125.23, 121.52, Math.toRadians(37.0)).mirror()
    private val shootPose = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpPPG2 = Pose(127.2, 84.0, Math.toRadians(0.0)).mirror()
    private val PPGtoShot = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpPGP1 = Pose(96.0, 57.0, Math.toRadians(0.0)).mirror()
    private val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0))
    private val pickUpPGP2 = Pose(135.8, 57.0, Math.toRadians(0.0))
    private val PGPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))
    private val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0)).mirror()
    private val pickUpGPP2 = Pose(134.3, 36.0, Math.toRadians(0.0)).mirror()
    private val GPPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    private val pickUpHP = Pose(132.27, 9.15, Math.toRadians(-90.0)).mirror()
    private val leavePoint = Pose(87.73, 110.42, Math.toRadians(45.0)).mirror()
    private val hitGate = Pose(135.51, 60.203, Math.toRadians(45.0)).mirror()
    private val hitGateControl = Pose(90.76, 61.42, Math.toRadians(45.0)).mirror()
    private lateinit var PPGfirst: PathChain
    private lateinit var PPGsecond: PathChain
    private lateinit var PPGtoShotMove: PathChain
    private lateinit var PGPfirst: PathChain
    private lateinit var PGPsecond: PathChain
    private lateinit var PGPtoShotMove: PathChain
    private lateinit var GPPfirst: PathChain
    private lateinit var GPPsecond: PathChain
    private lateinit var GPPtoShotMove: PathChain
    private lateinit var TheGate: PathChain
    private lateinit var Leave: PathChain
    private lateinit var GoToShot: PathChain

    private fun buildPaths() {

    GoToShot = follower.pathBuilder()
        .addPath(BezierLine(startPose, shootPose))
        .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
        .build()
    Leave = follower.pathBuilder()
        .addPath(BezierLine(pickUpGPP2, leavePoint))
        .setLinearHeadingInterpolation(startPose.heading, leavePoint.heading)
        .build()
    PPGfirst = follower.pathBuilder()
        .addPath(BezierLine(shootPose, pickUpPPG1))
        .setLinearHeadingInterpolation(shootPose.heading, pickUpPPG1.heading)
        .build()
    PPGsecond = follower.pathBuilder()
        .addPath(BezierLine(pickUpPPG1, pickUpPPG2))
        .setLinearHeadingInterpolation(pickUpPPG1.heading, pickUpPPG2.heading)
        .build()
    PPGtoShotMove = follower.pathBuilder()
        .addPath(BezierLine(pickUpPPG2, PPGtoShot))
        .setLinearHeadingInterpolation(pickUpPPG2.heading, PPGtoShot.heading)
        .build()
    //PGP paths
    PGPfirst = follower.pathBuilder()
        .addPath(BezierCurve(shootPose, pickUpPGPControl, pickUpPGP1))
        .setLinearHeadingInterpolation(shootPose.heading, pickUpPGP1.heading)
        .build()
    PGPsecond = follower.pathBuilder()
        .addPath(BezierLine(pickUpPGP1, pickUpPGP2))
        .setLinearHeadingInterpolation(pickUpPGP1.heading, pickUpPGP2.heading)
        .build()
    TheGate = follower.pathBuilder()
        .addPath(BezierCurve(PPGtoShot, hitGateControl, hitGate))
        .setLinearHeadingInterpolation(pickUpPGP2.heading, hitGate.heading)
        .build()
    PGPtoShotMove = follower.pathBuilder()
        .addPath(BezierLine(pickUpPGP2, PGPtoShot))
        .setLinearHeadingInterpolation(pickUpPGP2.heading, PGPtoShot.heading)
        .build()
    //GPP paths
    GPPfirst = follower.pathBuilder()
        .addPath(BezierLine(shootPose, pickUpGPP1))
        .setLinearHeadingInterpolation(shootPose.heading, pickUpGPP1.heading)
        .build()
    GPPsecond = follower.pathBuilder()
        .addPath(BezierLine(pickUpGPP1, pickUpGPP2))
        .setLinearHeadingInterpolation(pickUpGPP1.heading, pickUpGPP2.heading)
        .build()
    GPPtoShotMove = follower.pathBuilder()
        .addPath(BezierLine(pickUpGPP2, GPPtoShot))
        .setLinearHeadingInterpolation(pickUpGPP2.heading, GPPtoShot.heading)
        .build()

}




}
