package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.pedropathing.geometry.Pose
import dev.nextftc.control2.util.InterpolatingMap2D
import dev.nextftc.core.commands.CommandManager
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle

object Interpolation {

    private val flywheelVelocity = InterpolatingMap2D.bilinear()
    private val hood = InterpolatingMap2D.bilinear()
    private val airTime = InterpolatingMap2D.bilinear()

    init {

        // GRID AXES
        // X: 24, 48, 72, 96
        // Y: 117.97, 93.97, 69.97, 45.97, 21.97

        // FLYWHEEL VELOCITY

        // Exact raw matches
        flywheelVelocity[24.0, 117.97] = 1960.0
        flywheelVelocity[48.0, 117.97] = 1780.0
        flywheelVelocity[72.0, 117.97] = 1640.0
        flywheelVelocity[96.0, 117.97] = 1460.0

        // Exact raw matches
        flywheelVelocity[24.0, 93.97] = 1960.0
        flywheelVelocity[48.0, 93.97] = 1840.0
        flywheelVelocity[72.0, 93.97] = 1640.0
        flywheelVelocity[96.0, 93.97] = 1400.0

        // X=24 IDW calculated, rest are exact raw matches
        flywheelVelocity[24.0, 69.97] = 1854.0
        flywheelVelocity[48.0, 69.97] = 1820.0
        flywheelVelocity[72.0, 69.97] = 1720.0
        flywheelVelocity[96.0, 69.97] = 1780.0

        // X=72 is exact, rest are IDW calculated
        flywheelVelocity[24.0, 45.97] = 1894.0
        flywheelVelocity[48.0, 45.97] = 1908.0
        flywheelVelocity[72.0, 45.97] = 1860.0
        flywheelVelocity[96.0, 45.97] = 1858.0

        // X=72 exact. X=48 and X=96 heavily anchored by scattered points P16 and P14 respectively.
        flywheelVelocity[24.0, 21.97] = 2002.0
        flywheelVelocity[48.0, 21.97] = 2060.0
        flywheelVelocity[72.0, 21.97] = 2100.0
        flywheelVelocity[96.0, 21.97] = 1883.0


        // HOOD

        // Exact raw matches
        hood[24.0, 117.97] = 0.800
        hood[48.0, 117.97] = 0.550
        hood[72.0, 117.97] = 0.600
        hood[96.0, 117.97] = 0.300

        // Exact raw matches
        hood[24.0, 93.97] = 0.750
        hood[48.0, 93.97] = 0.750
        hood[72.0, 93.97] = 0.600
        hood[96.0, 93.97] = 0.350

        // X=24 IDW calculated, rest are exact raw matches
        hood[24.0, 69.97] = 0.700
        hood[48.0, 69.97] = 0.650
        hood[72.0, 69.97] = 0.700
        hood[96.0, 69.97] = 0.700

        // X=72 exact, rest are IDW calculated
        hood[24.0, 45.97] = 0.660
        hood[48.0, 45.97] = 0.667
        hood[72.0, 45.97] = 0.700
        hood[96.0, 45.97] = 0.697

        // X=72 exact. Bottom corners recalculated (anchored by P16 and P14 which had much lower angles)
        hood[24.0, 21.97] = 0.638
        hood[48.0, 21.97] = 0.551
        hood[72.0, 21.97] = 0.800
        hood[96.0, 21.97] = 0.652


        // AIR TIME - 9 measurement points in a 3x3 grid
        // Preserved original values since no truth data was provided

        // Close range (y = 117.97)
        airTime[24.0, 117.97] = 0.41
        airTime[60.0, 117.97] = 0.46
        airTime[96.0, 117.97] = 0.35

        // Mid range (y = 69.97)
        airTime[24.0, 69.97] = 0.46
        airTime[60.0, 69.97] = 0.50
        airTime[96.0, 69.97] = 0.39

        // Far range (y = 21.97)
        airTime[24.0, 21.97] = 0.78
        airTime[60.0, 21.97] = 0.79
        airTime[96.0, 21.97] = 0.81
    }

    fun getFlywheelVelocity(pose: Pose): Double {
        return flywheelVelocity[pose.x, pose.y]
    }

    fun getHood(pose: Pose): Double {
        return hood[pose.x, pose.y]
    }

    fun getAirTime(pose: Pose): Double {
        return airTime[pose.x, pose.y]
    }

    /**
     * Apply a shot by setting the hood angle and flywheel speed based on robot pose.
     */
    fun applyShot(pose: Pose) {
        val velocity = getFlywheelVelocity(pose)
        val angle = getHood(pose)

        ShooterAngle.targetPosition = angle + ShooterAngle.manualOffset
        CommandManager.scheduleCommand(ShooterAngle.update())
        CommandManager.scheduleCommand(Shooter.spinAtSpeed(velocity + Shooter.manualOffset))
    }
}