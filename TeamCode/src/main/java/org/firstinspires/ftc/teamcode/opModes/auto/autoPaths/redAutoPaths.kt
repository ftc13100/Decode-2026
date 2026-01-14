package org.firstinspires.ftc.teamcode.opModes.auto.autoPaths

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent

object redAutoPaths : Subsystem {
    ////////KEY//////////
    //pickUp_____1 == Getting to pick up motif row
    //pickUp_____2 == Ramming into the motif
    //___toShot    == Goes to the shootPose after picking up
    //_____Control == Dilates the straight line into a curve

    ///////DOCS//////////
    //Parallel Groups    == Everything within happens at the same time
    //Sequential Groups  == Everything within happens chronologically

     val startPose = Pose(125.23, 121.52, Math.toRadians(37.0))
     val shootPose = Pose(84.0, 84.0, Math.toRadians(0.0))
     val pickUpPPG1 = Pose(98.35, 84.0, Math.toRadians(0.0))
     val pickUpPPG2 = Pose(127.2, 84.0, Math.toRadians(0.0))
     val PPGtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))
     val pickUpPGP1 = Pose(96.0, 57.0, Math.toRadians(0.0))
     val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0))
     val pickUpPGP2 = Pose(135.8, 57.0, Math.toRadians(0.0))
     val PGPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))
     val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0))
     val pickUpGPP2 = Pose(134.3, 36.0, Math.toRadians(0.0))
     val GPPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))

     val gate = Pose(127.5, 75.90674955595027, Math.toRadians(125.0))
     val leavePoint = Pose(87.73, 110.42, Math.toRadians(45.0))
     val hitGate = Pose(135.51, 60.203, Math.toRadians(45.0))
     val hitGateControl = Pose(90.76, 61.42, Math.toRadians(45.0))
     val bottomStartPose = Pose(56.0, 7.5, Math.toRadians(90.0))
     val bottomShootPose = Pose(56.0, 10.5, Math.toRadians(90.0))
     val bottomLeavePoint = Pose(36.49261083743842, 8.20935960591133, Math.toRadians(90.0))

     lateinit var PPGfirst: PathChain
     lateinit var PPGsecond: PathChain
     lateinit var PPGtoShotMove: PathChain
     lateinit var PGPfirst: PathChain
     lateinit var PGPsecond: PathChain
     lateinit var PGPtoShotMove: PathChain
     lateinit var GPPfirst: PathChain
     lateinit var GPPsecond: PathChain
     lateinit var GPPtoShotMove: PathChain
     lateinit var TheGate: PathChain
     lateinit var Leave: PathChain
     lateinit var GoToShot: PathChain
     lateinit var MohitHitGate: PathChain
     lateinit var bottomShoot: PathChain
     lateinit var bottomLeave: PathChain

     fun buildPaths() {
        bottomShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomShootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        bottomLeave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomLeavePoint))
            .setLinearHeadingInterpolation(shootPose.heading, leavePoint.heading)
            .build()
        GoToShot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        Leave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP2, leavePoint))
            .setLinearHeadingInterpolation(startPose.heading, leavePoint.heading)
            .build()
        PPGfirst = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(shootPose, pickUpPPG1))
            .setLinearHeadingInterpolation(shootPose.heading, pickUpPPG1.heading)
            .build()
        PPGsecond = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG1, pickUpPPG2))
            .setLinearHeadingInterpolation(pickUpPPG1.heading, pickUpPPG2.heading)
            .build()
        PPGtoShotMove = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG2, PPGtoShot))
            .setLinearHeadingInterpolation(pickUpPPG2.heading, PPGtoShot.heading)
            .build()
        PGPfirst = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(shootPose, pickUpPGPControl, pickUpPGP1))
            .setLinearHeadingInterpolation(shootPose.heading, pickUpPGP1.heading)
            .build()
        PGPsecond = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP1, pickUpPGP2))
            .setLinearHeadingInterpolation(pickUpPGP1.heading, pickUpPGP2.heading)
            .build()
        TheGate = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(PPGtoShot, hitGateControl, hitGate))
            .setLinearHeadingInterpolation(pickUpPGP2.heading, hitGate.heading)
            .build()
        PGPtoShotMove = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpPGP2, PGPtoShot))
            .setLinearHeadingInterpolation(pickUpPGP2.heading, PGPtoShot.heading)
            .build()
        GPPfirst = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(shootPose, pickUpGPP1))
            .setLinearHeadingInterpolation(shootPose.heading, pickUpGPP1.heading)
            .build()
        GPPsecond = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP1, pickUpGPP2))
            .setLinearHeadingInterpolation(pickUpGPP1.heading, pickUpGPP2.heading)
            .build()
        GPPtoShotMove = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP2, GPPtoShot))
            .setLinearHeadingInterpolation(pickUpGPP2.heading, GPPtoShot.heading)
            .build()
        MohitHitGate = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpPPG2, gate))
            .setLinearHeadingInterpolation(pickUpPPG2.heading,gate.heading)
            .build()

    }
}