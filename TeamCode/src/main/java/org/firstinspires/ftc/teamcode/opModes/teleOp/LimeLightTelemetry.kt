package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose

@TeleOp(name = "LimeLight with more Telemetry")
class LimeLightTelemetry : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(),
            BindingsComponent,
            BulkReadComponent,
        )
    }

    private val frontLeftName = "leftFront"
    private val frontRightName = "rightFront"
    private val backLeftName = "leftRear"
    private val backRightName = "rightRear"

    private lateinit var frontLeftMotor: MotorEx
    private lateinit var frontRightMotor: MotorEx
    private lateinit var backLeftMotor: MotorEx
    private lateinit var backRightMotor: MotorEx

    private lateinit var driverControlled: MecanumDriverControlled

    private lateinit var limelight: Limelight3A

    private lateinit var follower: Follower

    override fun onInit() {
        frontLeftMotor = MotorEx(frontLeftName).reversed()
        frontRightMotor = MotorEx(frontRightName)
        backLeftMotor = MotorEx(backLeftName).reversed()
        backRightMotor = MotorEx(backRightName)

        listOf(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        telemetry.msTransmissionInterval = 11
        limelight.pipelineSwitch(1)
        limelight.start()

        // --- Pedro Pathing Follower Init ---
        // NOTE: You must replace 'Constants.createFollower(hardwareMap)'
        // with the actual way you initialize the Follower in your project.
        // I am using the method from your 'Example' code for consistency.
        // Ensure you have access to the Pedro Pathing 'Constants' setup.
        try {
            // Assuming you have a Constants object with a createFollower method
            val constantsClass = Class.forName("org.firstinspires.ftc.teamcode.pedroPathing.Constants")
            val createFollowerMethod = constantsClass.getMethod("createFollower", com.qualcomm.robotcore.hardware.HardwareMap::class.java)
            follower = createFollowerMethod.invoke(null, hardwareMap) as Follower
        } catch (e: Exception) {
            // Fallback initialization or error handling if Constants.createFollower is not available
            telemetry.log().add("Error initializing Follower: ${e.message}")
            // Consider throwing a RuntimeException or initializing with a mock/simple Follower if possible
        }

        // Use a default starting pose for teleop if not set by an auto
        follower.setStartingPose(Pose())
        follower.update()
    }

    override fun onStartButtonPressed() {
        // ... (existing driver controlled setup)
        driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled.scalar = 1.0

        button { gamepad1.y }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { driverControlled.scalar = 0.4 }
            .whenBecomesFalse { driverControlled.scalar = 1.0 }

        // Start the follower's teleop drive mode if desired (it's optional
        // since NextFTC handles the motor control)
        // follower.startTeleopDrive()
    }

    override fun onUpdate() {
        BindingManager.update()
        driverControlled.update()

        // --- CRUCIAL: Update the follower's odometry at the start of the loop ---
        follower.update()

        // Limelight data (AprilTag Vision)
        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val botpose: Pose3D = result.botpose
            telemetry.addData("LL_tx", result.tx)
            telemetry.addData("LL_ty", result.ty)
            telemetry.addData("LL_Botpose", botpose.toString())
        }

        // --- Robot Position Data (Odometry) ---
        val robotPose: Pose = follower.pose
        val headingDegrees = Math.toDegrees(robotPose.heading)

        telemetry.addLine("*** Robot Pose (Odometry) ***")
        telemetry.addData("OD_X", "%.2f", robotPose.x)
        telemetry.addData("OD_Y", "%.2f", robotPose.y)
        telemetry.addData("OD_Heading", "%.2f deg", headingDegrees)
        telemetry.addLine("*****************************")

        telemetry.addData("Mode", "TeleOp Running")
        telemetry.update()
    }

    override fun onStop() {
        BindingManager.reset()
    }
}