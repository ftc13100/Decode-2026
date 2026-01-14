package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

object blueAutoPaths : Subsystem {
    ////////KEY//////////
    //pickUp_____1 == Getting to pick up motif row
    //pickUp_____2 == Ramming into the motif
    //___toShot    == Goes to the shootPose after picking up
    //_____Control == Dilates the straight line into a curve

    ///////DOCS//////////
    //Parallel Groups    == Everything within happens at the same time
    //Sequential Groups  == Everything within happens chronologically
    public val startPose = Pose(125.23, 121.52, Math.toRadians(37.0)).mirror()
    public val shootPose = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    public val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0)).mirror()
    public val pickUpPPG2 = Pose(127.2, 84.0, Math.toRadians(0.0)).mirror()
    public val PPGtoShot = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    public val pickUpPGP1 = Pose(96.0, 57.0, Math.toRadians(0.0)).mirror()
    public val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0)).mirror()
    public val pickUpPGP2 = Pose(135.8, 57.0, Math.toRadians(0.0)).mirror()
    public val PGPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()
    public val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0)).mirror()
    public val pickUpGPP2 = Pose(134.3, 36.0, Math.toRadians(0.0)).mirror()
    public val GPPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0)).mirror()

    public val gate = Pose(127.5, 75.90674955595027, Math.toRadians(125.0))

    public val pickUpHP = Pose(132.27, 9.15, Math.toRadians(-90.0)).mirror()
    public val leavePoint = Pose(87.73, 110.42, Math.toRadians(45.0)).mirror()
    public val hitGate = Pose(135.51, 60.203, Math.toRadians(45.0)).mirror()
    public val hitGateControl = Pose(90.76, 61.42, Math.toRadians(45.0)).mirror()

    public val bottomstartPose = Pose(56.0, 7.5, Math.toRadians(90.0))
    public val bottomshootPose = Pose(56.0, 10.5, Math.toRadians(90.0))

    public val bottomleavePoint = Pose(36.49261083743842, 8.20935960591133, Math.toRadians(90.0))
    
    public lateinit var PPGfirst: PathChain
    public lateinit var PPGsecond: PathChain
    public lateinit var PPGtoShotMove: PathChain
    public lateinit var PGPfirst: PathChain
    public lateinit var PGPsecond: PathChain
    public lateinit var PGPtoShotMove: PathChain
    public lateinit var GPPfirst: PathChain
    public lateinit var GPPsecond: PathChain
    public lateinit var GPPtoShotMove: PathChain
    public lateinit var TheGate: PathChain
    public lateinit var Leave: PathChain
    public lateinit var GoToShot: PathChain
    public lateinit var MohitHitGate: PathChain
    public lateinit var bottomshoot: PathChain
    public lateinit var bottomLeave: PathChain





    public fun buildPaths() {
    bottomshoot = follower.pathBuilder()
        .addPath(BezierLine(bottomstartPose, bottomshootPose))
        .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
        .build()
    bottomLeave = follower.pathBuilder()
         .addPath(BezierLine(bottomstartPose, bottomleavePoint))
         .setLinearHeadingInterpolation(shootPose.heading, leavePoint.heading)
         .build()


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
    MohitHitGate = follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG2, gate))
            .setLinearHeadingInterpolation(pickUpPPG2.heading,gate.heading)
            .build()

}




}
