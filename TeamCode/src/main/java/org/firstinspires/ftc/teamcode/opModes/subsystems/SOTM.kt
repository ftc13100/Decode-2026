package org.firstinspires.ftc.teamcode.opModes.subsystems
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.Subsystem

object SOTM : Subsystem {

    private const val ITERATIONS = 5

    fun calculateVirtualRobot(robotPose: Vector, robotVelocity: Vector): Vector? {
        var virtualPose: Vector? = robotPose
        for (i in 0..<ITERATIONS) {
            // airtime from BiLinear Shooter table
            //            val airTime: Double = Interpolation.getAirTime(virtualPose)
            // virtualPose = robotPose.plus(robotVelocity.times(airTime))
        }
        return virtualPose
    }
}