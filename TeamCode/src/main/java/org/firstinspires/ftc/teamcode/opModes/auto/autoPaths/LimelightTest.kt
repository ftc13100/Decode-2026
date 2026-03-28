
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage

class LimelightTest : LinearOpMode() {
    private var limelight: Limelight3A? = null

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        limelight = hardwareMap.get<Limelight3A?>(Limelight3A::class.java, "limelight")

        telemetry.setMsTransmissionInterval(11)

        limelight?.pipelineSwitch(0)

        /*
         * Starts polling for data.
         */
        limelight!!.start()

        while (opModeIsActive()) {
            val result = limelight!!.getLatestResult()
            if (result != null) {
                if (result.isValid()) {
                    val tx = result.tx
                    telemetry.addData("tx", result.getTx())
                    telemetry.addData("ty", result.getTy())
                }
            }
        }
    }
}