/*package org.firstinspires.ftc.teamcode.auto
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants


@Autonomous(name = "PedroPath")
class blueBottom: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)


        )
    }
    //starting position

    private val startPose = Pose(85.5, 8.3, Math.toRadians(90.0))
    private val depositPose = Pose(84.3, 61.9, Math.toRadians(0.0))

    private val curvePoint = Pose(138.2, 48.1, Math.toRadians(45.0))

    private lateinit var skib: PathChain


    private fun buildPaths() {

        skib = follower.pathBuilder()
            .addPath(BezierCurve(Point(startPose), Point(curvePoint), Point(depositPose)))
            .setLinearHeadingInterpolation(startPose.heading, depositPose.heading)
            .build()
    }





    val secondRoutine: Command
        get() = SequentialGroup(
            FollowPath.(skib)
        )

    override fun onInit() {
        follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)

        follower.setMaxPower(0.7)
        follower.setStartingPose(//startPose)
        buildPaths()
    }

    override fun onStartButtonPressed() {
        secondRoutine()
    }
}

 */