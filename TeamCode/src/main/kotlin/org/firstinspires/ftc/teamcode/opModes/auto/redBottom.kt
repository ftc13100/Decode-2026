package org.firstinspires.ftc.teamcode.auto
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.ftc.NextFTCOpMode


@Autonomous(name = "redBottom")
class redBottom: NextFTCOpMode() {
    //starting position



    private fun buildPaths() {
    }





    val secondRoutine: Command
        get() = SequentialGroup(

        )

    override fun onInit() {
       // follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
        //follower.setMaxPower(0.7)
        //follower.setStartingPose(//startPose)
       // buildPaths()
    }

    override fun onStartButtonPressed() {
        secondRoutine()
    }
}