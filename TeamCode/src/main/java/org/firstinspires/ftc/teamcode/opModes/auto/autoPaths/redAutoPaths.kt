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
     val pushStart = Pose(62.5615763546798, 9.596059113300495, Math.toRadians(90.0)).mirror()
     val dopush = Pose(47.665024630541865,   9.753694581280788, Math.toRadians(90.0)).mirror()
     val pushToShoot = Pose(60.0, 84.0, Math.toRadians(180.0)).mirror()
     val pickUpPPG1 = Pose(98.35, 85.0, Math.toRadians(0.0))
     val pickUpPPG2 = Pose(126.5, 85.0, Math.toRadians(0.0))
     val PPGtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))
     val pickUpPGP1 = Pose(96.0, 57.0, Math.toRadians(0.0))
     val pickUpPGPControl = Pose(84.7, 57.6, Math.toRadians(0.0))
     val pickUpPGP2 = Pose(134.0, 57.0, Math.toRadians(0.0))
     val PGPtoShot = Pose(84.0, 84.0, Math.toRadians(0.0))
     val pickUpGPP1 = Pose(98.25, 36.0, Math.toRadians(0.0))
     val pickUpGPP2 = Pose(131.3, 36.0, Math.toRadians(0.0))
     val GPPtoShot = Pose(85.16523235800345, 107.3803786574871, Math.toRadians(34.0))

     val gate = Pose(125.0271186440678, 78.65254237288136, Math.toRadians(110.0))

     val secretTunnel = Pose(130.3050847457627, 55.983050847457626, Math.toRadians(0.0))

    val leavePoint = Pose(87.73, 110.42, Math.toRadians(40.67))
     val hitGate = Pose(134.91525423728814, 59.79661016949153, Math.toRadians(45.0))

    val eat = Pose(134.91525423728814, 57.79661016949153, Math.toRadians(45.0))

    val hitGateControl = Pose(89.31958762886597, 70.76288659793815, Math.toRadians(45.0))
    val bottomStartPose = Pose(56.0, 8.0, Math.toRadians(180.0)).mirror()
    val bottomHPpose = Pose(9.0, 10.0, Math.toRadians(170.0)).mirror()
    val bottomHPviggle = Pose(19.0, 10.0, Math.toRadians(180.0)).mirror()
    val bottomHPpose2 = Pose(9.0, 10.0, Math.toRadians(190.0)).mirror()

    val bottomintakepose = Pose(9.0, 10.0, Math.toRadians(170.0)).mirror()
    val bottomintakeviggle = Pose(19.0, 10.0, Math.toRadians(180.0)).mirror()
    val bottomintakepose2 = Pose(9.0, 10.0, Math.toRadians(190.0)).mirror()

    val bottomSpikemark = Pose(8.0, 35.8, Math.toRadians(180.0)).mirror()
    val bottomSpikemarkControl = Pose(58.0, 39.7, Math.toRadians(180.0)).mirror()

    val bottomLeavePoint = Pose(36.667814113597245, 14.196213425129093, Math.toRadians(180.0)).mirror()

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

    lateinit var GoToSecretTunnel: PathChain

    lateinit var Leave: PathChain
     lateinit var GoToShot: PathChain
     lateinit var MohitHitGate: PathChain
     lateinit var bottomShoot: PathChain
     lateinit var bottomLeave: PathChain

     lateinit var DeadhuzzLeave: PathChain

    lateinit var eatup: PathChain
    lateinit var push: PathChain
    lateinit var pushShoot: PathChain

    lateinit var bottomHP: PathChain
    lateinit var HPshoot: PathChain
    lateinit var HPviggle: PathChain
    lateinit var HPviggleagain: PathChain
    lateinit var bottomIntake: PathChain
    lateinit var intakeShoot: PathChain
    lateinit var bottomSpikeGet: PathChain
    lateinit var bottomSpikeGetBack: PathChain


    lateinit var bottomIntake2: PathChain
    lateinit var HPviggletoShoot: PathChain

    lateinit var  bottomIntake2toShoot: PathChain





    fun buildPaths() {
        push= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pushStart, dopush))
            .setLinearHeadingInterpolation(pushStart.heading, dopush.heading)
            .build()
        pushShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(dopush, pushToShoot))
            .setLinearHeadingInterpolation(dopush.heading, pushToShoot.heading)
            .build()
        bottomHP = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomHPpose))
            .setTangentHeadingInterpolation()
            .build()
        HPviggle = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomHPpose, bottomHPviggle))
            .setLinearHeadingInterpolation(bottomHPpose.heading, bottomHPviggle.heading)
            .build()
        HPviggleagain = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomHPviggle, bottomHPpose2))
            .setLinearHeadingInterpolation(bottomHPviggle.heading, bottomHPpose2.heading)
            .build()
        HPviggletoShoot= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomHPpose2, bottomStartPose))
            .setLinearHeadingInterpolation(bottomHPpose2.heading, bottomStartPose.heading)
            .build()
        intakeShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomintakepose))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomintakepose.heading)
            .build()
        bottomIntake = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomintakepose, bottomintakeviggle))
            .setLinearHeadingInterpolation(bottomintakepose.heading, bottomintakeviggle.heading)
            .build()
        bottomIntake2 = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomintakeviggle, bottomintakepose2))
            .setLinearHeadingInterpolation(bottomintakeviggle.heading, bottomintakepose2.heading)
            .build()
        bottomIntake2toShoot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomintakepose2, bottomStartPose))
            .setLinearHeadingInterpolation(bottomintakepose2.heading, bottomStartPose.heading)
            .build()
        bottomLeave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomStartPose, bottomLeavePoint))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomLeavePoint.heading)
            .build()
        bottomSpikeGet = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(bottomStartPose, bottomSpikemarkControl, bottomSpikemark))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomSpikemark.heading)
            .build()
        bottomSpikeGetBack = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(bottomSpikemark, bottomStartPose))
            .setLinearHeadingInterpolation(bottomStartPose.heading, bottomSpikemark.heading)
            .build()
        GoToShot = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        Leave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pickUpGPP2, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
            .build()
        Leave = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(hitGate, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, shootPose.heading)
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
            .setLinearHeadingInterpolation(PPGtoShot.heading, hitGate.heading)
            .build()
        eatup = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(hitGate, eat))
            .setLinearHeadingInterpolation(hitGate.heading, eat.heading)
            .build()

        PGPtoShotMove = PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(pickUpPGP2, pickUpPGPControl, PGPtoShot))
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
        DeadhuzzLeave = PedroComponent.Companion.follower.pathBuilder()
             .addPath(BezierLine(shootPose, leavePoint))
             .setLinearHeadingInterpolation(shootPose.heading,leavePoint.heading)
             .build()

    }
}