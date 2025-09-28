package org.firstinspires.ftc.teamcode.opModes.testing
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

@TeleOp(name = "LimelightTest")
class LimelightTest : LinearOpMode() {

    private lateinit var limelight: Limelight3A

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        telemetry.msTransmissionInterval = 11

        limelight.pipelineSwitch(1)

        // Starts polling for data
        limelight.start()

        waitForStart()

        while (opModeIsActive()) {
            val result: LLResult? = limelight.latestResult
            if (result != null && result.isValid) {
                val botpose: Pose3D = result.botpose
                telemetry.addData("tx", result.tx)
                telemetry.addData("ty", result.ty)
                telemetry.addData("Botpose", botpose.toString())
            }
            telemetry.update()
        }
    }
}