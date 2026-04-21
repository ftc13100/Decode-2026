package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.control2.util.InterpolatingMap2D

object Interpolation {

    private val flywheelVelocity = InterpolatingMap2D.bilinear()
    private val hood = InterpolatingMap2D.bilinear()
//    private val airTime = InterpolatingMap2D.bilinear()

    init {

        // GRID AXES
        // X: 24, 48, 72, 96
        // Y: 117.97, 93.97, 69.97, 45.97, 21.97


        // FLYWHEEL VELOCITY

        flywheelVelocity[24.0, 117.97] = 1960.0
        flywheelVelocity[48.0, 117.97] = 1780.0
        flywheelVelocity[72.0, 117.97] = 1640.0
        flywheelVelocity[96.0, 117.97] = 1460.0

        flywheelVelocity[24.0, 93.97] = 1960.0
        flywheelVelocity[48.0, 93.97] = 1840.0
        flywheelVelocity[72.0, 93.97] = 1640.0
        flywheelVelocity[96.0, 93.97] = 1400.0

        flywheelVelocity[24.0, 69.97] = 1890.0
        flywheelVelocity[48.0, 69.97] = 1820.0
        flywheelVelocity[72.0, 69.97] = 1720.0
        flywheelVelocity[96.0, 69.97] = 1780.0

        flywheelVelocity[24.0, 45.97] = 1980.0
        flywheelVelocity[48.0, 45.97] = 1920.0
        flywheelVelocity[72.0, 45.97] = 1860.0
        flywheelVelocity[96.0, 45.97] = 1900.0

        flywheelVelocity[24.0, 21.97] = 2060.0
        flywheelVelocity[48.0, 21.97] = 2060.0
        flywheelVelocity[72.0, 21.97] = 2100.0
        flywheelVelocity[96.0, 21.97] = 2080.0

        // HOOD

        hood[24.0, 117.97] = 0.800
        hood[48.0, 117.97] = 0.550
        hood[72.0, 117.97] = 0.600
        hood[96.0, 117.97] = 0.300

        hood[24.0, 93.97] = 0.750
        hood[48.0, 93.97] = 0.750
        hood[72.0, 93.97] = 0.600
        hood[96.0, 93.97] = 0.350

        hood[24.0, 69.97] = 0.700
        hood[48.0, 69.97] = 0.650
        hood[72.0, 69.97] = 0.700
        hood[96.0, 69.97] = 0.700

        hood[24.0, 45.97] = 0.720
        hood[48.0, 45.97] = 0.710
        hood[72.0, 45.97] = 0.700
        hood[96.0, 45.97] = 0.690

        hood[24.0, 21.97] = 0.820
        hood[48.0, 21.97] = 0.810
        hood[72.0, 21.97] = 0.800
        hood[96.0, 21.97] = 0.780

        // AIR TIME
//
//        airTime[24.0, 117.97] = 0.41
//        airTime[48.0, 117.97] = 0.54
//        airTime[72.0, 117.97] = 0.38
//        airTime[96.0, 117.97] = 0.35
//
//        airTime[24.0, 93.97] = 0.44
//        airTime[48.0, 93.97] = 0.38
//        airTime[72.0, 93.97] = 0.27
//        airTime[96.0, 93.97] = 0.35
//
//        airTime[24.0, 69.97] = 0.46
//        airTime[48.0, 69.97] = 0.50
//        airTime[72.0, 69.97] = 0.42
//        airTime[96.0, 69.97] = 0.39
//
//        airTime[24.0, 45.97] = 0.55
//        airTime[48.0, 45.97] = 0.60
//        airTime[72.0, 45.97] = 0.70
//        airTime[96.0, 45.97] = 0.68
//
//        airTime[24.0, 21.97] = 0.78
//        airTime[48.0, 21.97] = 0.79
//        airTime[72.0, 21.97] = 0.80
//        airTime[96.0, 21.97] = 0.81
    }

    fun getFlywheelVelocity(pose: Pose): Double {
        return flywheelVelocity[pose.x, pose.y]
    }

    fun getHood(pose: Pose): Double {
        return hood[pose.x, pose.y]
    }

//    fun getAirTime(pose: Pose): Double {
//        return airTime[pose.x, pose.y]
//    }

}