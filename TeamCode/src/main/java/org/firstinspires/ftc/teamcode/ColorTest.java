package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "colortest", group = "Test")
public class ColorTest extends LinearOpMode {
    private static final Color INDIGO = new Color(75, 0, 130);
    private static final Color VIOLET = new Color(143, 0, 255);

    private static final Color[] COLORS = {
            Color.RED,
            Color.ORANGE,
            Color.YELLOW,
            Color.GREEN,
            Color.BLUE,
            INDIGO,
            VIOLET
    };

    private static final String[] COLOR_NAMES = {
            "Red",
            "Orange",
            "Yellow",
            "Green",
            "Blue",
            "Indigo",
            "Violet"
    };

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "fikerled");
        prism.setStripLength(36);

        int index = 0;
        boolean prevRight = false;
        boolean prevLeft = false;

        waitForStart();

        setColor(prism, index);

        while (opModeIsActive()) {
            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;

            if (right && !prevRight) {
                index = (index + 1) % COLORS.length;
                setColor(prism, index);
            }

            if (left && !prevLeft) {
                index = (index - 1 + COLORS.length) % COLORS.length;
                setColor(prism, index);
            }

            prevRight = right;
            prevLeft = left;

            telemetry.addData("Color", COLOR_NAMES[index]);
            telemetry.addLine("DPad Right: Next");
            telemetry.addLine("DPad Left: Previous");
            telemetry.update();

            idle();
        }
    }

    private void setColor(GoBildaPrismDriver prism, int index) {
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Solid(COLORS[index], 0, 35)
        );
    }
}

