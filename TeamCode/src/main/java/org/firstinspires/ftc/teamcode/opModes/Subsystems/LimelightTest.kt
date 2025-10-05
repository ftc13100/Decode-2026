package org.firstinspires.ftc.teamcode.opModes.Subsystems
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.opModeIsActive
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

object LimelightTest : Subsystem {

    override fun initialize() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")

        limelight.pipelineSwitch(1)

        limelight.start()
    }
    lateinit var limelight: Limelight3A






    }
