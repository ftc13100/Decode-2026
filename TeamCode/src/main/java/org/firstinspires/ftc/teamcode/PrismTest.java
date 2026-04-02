package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "Prism Test", group = "Test")
public class PrismTest extends LinearOpMode {
    private static final int LIGHT_1_START = 0;
    private static final int LIGHT_1_END = 11;
    private static final int LIGHT_2_START = 12;
    private static final int LIGHT_2_END = 17;
    private static final int LIGHT_3_START = 18;
    private static final int LIGHT_3_END = 23;
    private static final int LIGHT_4_START = 24;
    private static final int LIGHT_4_END = 35;

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "fikerled");
        prism.setStripLength(36);

        waitForStart();

        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Rainbow()
        );

        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevY = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadRight = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !prevA) {
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Rainbow()
                );
            }

            if (gamepad1.b && !prevB) {
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Blink()
                );
            }

            if (gamepad1.x && !prevX) {
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(Color.GREEN)
                );
            }

            if (gamepad1.y && !prevY) {
                prism.clearAllAnimations();
            }

            if (gamepad1.dpad_up && !prevDpadUp) {
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Sparkle()
                );
            }

            if (gamepad1.dpad_down && !prevDpadDown) {
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Snakes()
                );
            }

            if (gamepad1.dpad_right && !prevDpadRight) {
                prism.clearAllAnimations();
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(Color.GREEN, LIGHT_1_START, LIGHT_1_END)
                );
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_1,
                        new PrismAnimations.Solid(Color.BLUE, LIGHT_2_START, LIGHT_2_END)
                );
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_2,
                        new PrismAnimations.Solid(Color.RED, LIGHT_3_START, LIGHT_3_END)
                );
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_3,
                        new PrismAnimations.Solid(Color.YELLOW, LIGHT_4_START, LIGHT_4_END)
                );
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadRight = gamepad1.dpad_right;

            telemetry.addLine("A: Rainbow  B: Blink  X: Solid Green");
            telemetry.addLine("DPad Up: Sparkle  DPad Down: Snakes  Y: Clear");
            telemetry.addLine("DPad Right: L1 Green  L2 Blue  L3 Red  L4 Yellow");
            telemetry.update();
            idle();
        }
    }
}

