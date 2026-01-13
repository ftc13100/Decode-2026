package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.subsystems.Subsystem
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


}
