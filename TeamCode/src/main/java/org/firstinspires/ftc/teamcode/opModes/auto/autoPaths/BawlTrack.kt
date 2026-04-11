
package org.firstinspires.ftc.teamcode.opModes.auto.blue

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack
import org.firstinspires.ftc.teamcode.opModes.subsystems.LimeLight.MohitBallTrack.limelight
import org.firstinspires.ftc.teamcode.opModes.subsystems.PoseStorage
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.ShooterAngle
import org.firstinspires.ftc.teamcode.opModes.subsystems.shooter.TurretAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Autonomous(name = "BawlTrack")
class BawlTrack: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                MohitBallTrack
            ),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }
    private val limelight: Limelight3A by lazy { hardwareMap.get(Limelight3A::class.java, "limelight") }
    override fun onInit() {
        telemetry.setMsTransmissionInterval(11)
        limelight.pipelineSwitch(0)
        limelight.start()
    }
    override fun onStartButtonPressed() {
        val result = limelight!!.getLatestResult()
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("tx", result.tx)
                telemetry.addData("ty", result.ty)
            }}}}

