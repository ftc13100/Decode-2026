package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDSubsystem {
    // if we ever add in another led strip we can change that stuff here
    private static final int STRIP_LENGTH = 12;
    private static final int FIRST_LED = 0;
    private static final int LAST_LED = STRIP_LENGTH - 1;

    private enum ActiveMode {
        NONE,
        SPIX,
        INTAKE
    }

    private final GoBildaPrismDriver prism;
    private ActiveMode lastMode = ActiveMode.NONE;
    private int lastSpindexerCount = -1;
    private Boolean lastIntakeRunning = null;

    public LEDSubsystem(GoBildaPrismDriver prism) {
        this.prism = prism;
        this.prism.setStripLength(STRIP_LENGTH);
    }

    /**
     * spindexer led states (can change if mohit and jackson wanna):
     * 0 = red, 1 = blue, 2 = yellow, 3 = green.
     */
    public void setSpindexerLights(int ballCount) {
        int count = Math.max(0, Math.min(ballCount, 3));
        if (lastMode == ActiveMode.SPIX && lastSpindexerCount == count) {
            return;
        }

        Color color;
        if (count == 0) {
            color = Color.RED;
        } else if (count == 1) {
            color = Color.BLUE;
        } else if (count == 2) {
            color = Color.YELLOW;
        } else {
            color = Color.GREEN;
        }

        prism.setSolidColor(FIRST_LED, LAST_LED, color);
        prism.show();

        lastMode = ActiveMode.SPIX;
        lastSpindexerCount = count;
        lastIntakeRunning = null;
    }

    /**
     * intake led modes:
     *
     * running = flashing
     * stopped = solid white.
     */
    public void setIntakeLights(boolean running) {
        if (lastMode == ActiveMode.INTAKE && lastIntakeRunning != null && lastIntakeRunning == running) {
            return;
        }

        prism.clearAllAnimations();

        if (running) {
            PrismAnimations.Blink blink = new PrismAnimations.Blink(Color.WHITE, Color.TRANSPARENT, 500, 250);
            blink.setStartIndex(FIRST_LED);
            blink.setStopIndex(LAST_LED);
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blink);
        } else {
            prism.setSolidColor(FIRST_LED, LAST_LED, Color.WHITE);
            prism.show();
        }

        lastMode = ActiveMode.INTAKE;
        lastIntakeRunning = running;
        lastSpindexerCount = -1;
    }

    public void clear() {
        prism.setSolidColor(FIRST_LED, LAST_LED, Color.TRANSPARENT);
        prism.show();
        lastMode = ActiveMode.NONE;
        lastSpindexerCount = -1;
        lastIntakeRunning = null;
    }

    //stuff that copilot told me to add so "legacy wrappers still work" i didnt really understand it
    public void setLights(int mode) {
        setSpindexerLights(mode);
    }

    public void setDecorative() {
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Rainbow()
        );
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_1,
                new PrismAnimations.Sparkle()
        );
        lastMode = ActiveMode.NONE;
    }

    public void setLights(int index, int colorCode) {
        if (index < 0 || index > 2) {
            return;
        }

        Color color;
        if (colorCode == 0) {
            color = Color.TRANSPARENT;
        } else if (colorCode == 1) {
            color = Color.PURPLE;
        } else if (colorCode == 2) {
            color = Color.GREEN;
        } else {
            return;
        }

        int start = index * 4;
        int end = Math.min(start + 3, LAST_LED);
        prism.setSolidColor(start, end, color);
        prism.show();
        lastMode = ActiveMode.NONE;
    }
}

