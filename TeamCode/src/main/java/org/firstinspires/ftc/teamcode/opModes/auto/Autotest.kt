//import com.qualcomm.hardware.limelightvision.LLResult
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import dev.nextftc.core.components.SubsystemComponent
//import dev.nextftc.ftc.ActiveOpMode
//import dev.nextftc.ftc.NextFTCOpMode
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
//import org.firstinspires.ftc.teamcode.opModes.subsystems.LimelightTest
//import org.firstinspires.ftc.teamcode.opModes.subsystems.LimelightTest.limelight
//
//@Autonomous(name = "Autotest")
//class Autotest : NextFTCOpMode() {
//    init {
//        addComponents(
//            SubsystemComponent(LimelightTest)
//        )
//    }
//    override fun onStartButtonPressed() {
//        while (opModeIsActive()) {
//            telemetry.msTransmissionInterval = 11
//            val result: LLResult? = limelight.latestResult
//            if (result != null && result.isValid) {
//                val botpose: Pose3D = result.botpose
//                ActiveOpMode.telemetry.addData("tx", result.tx)
//                ActiveOpMode.telemetry.addData("ty", result.ty)
//                ActiveOpMode.telemetry.addData("Botpose", botpose.toString())
//            }
//            ActiveOpMode.telemetry.update()
//        }
//    }
//}