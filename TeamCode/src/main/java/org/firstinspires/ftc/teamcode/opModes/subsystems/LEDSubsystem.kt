package org.firstinspires.ftc.teamcode.opModes.subsystems

import org.firstinspires.ftc.teamcode.opModes.subsystems.Prism.*
import kotlin.math.max
import kotlin.math.min

class LEDSubsystem(prism: GoBildaPrismDriver) {
    private enum class ActiveMode {
        NONE,
        SPIX,
        INTAKE
    }

    private val prism: GoBildaPrismDriver
    private var lastMode = ActiveMode.NONE
    private var lastSpindexerCount = -1
    private var lastIntakeRunning: Boolean? = null

    init {
        this.prism = prism
        this.prism.setStripLength(STRIP_LENGTH)
    }

    /**
     * spindexer led states (can change if mohit and jackson wanna):
     * 0 = red, 1 = blue, 2 = yellow, 3 = green.
     */
    fun setSpindexerLights(ballCount: Int) {
        val count = max(0, min(ballCount, 3))
        if (lastMode == ActiveMode.SPIX && lastSpindexerCount == count) {
            return
        }

        val color: Color?
        if (count == 0) {
            color = Color.RED
        } else if (count == 1) {
            color = Color.BLUE
        } else if (count == 2) {
            color = Color.YELLOW
        } else {
            color = Color.GREEN
        }

        prism.setSolidColor(FIRST_LED, LAST_LED, color)
        prism.show()

        lastMode = ActiveMode.SPIX
        lastSpindexerCount = count
        lastIntakeRunning = null
    }

    /**
     * intake led modes:
     *
     * running = flashing
     * stopped = solid white.
     */
    fun setIntakeLights(running: Boolean) {
        if (lastMode == ActiveMode.INTAKE && lastIntakeRunning != null && lastIntakeRunning == running) {
            return
        }

        prism.clearAllAnimations()

        if (running) {
            val blink: PrismAnimations.Blink = PrismAnimations.Blink(
                Color.WHITE,
                Color.TRANSPARENT,
                500,
                250
            )
            blink.setStartIndex(FIRST_LED)
            blink.setStopIndex(LAST_LED)
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blink)
        } else {
            prism.setSolidColor(
                FIRST_LED,
                LAST_LED,
                Color.WHITE
            )
            prism.show()
        }

        lastMode = ActiveMode.INTAKE
        lastIntakeRunning = running
        lastSpindexerCount = -1
    }

    fun clear() {
        prism.setSolidColor(
            FIRST_LED,
            LAST_LED,
            Color.TRANSPARENT
        )
        prism.show()
        lastMode = ActiveMode.NONE
        lastSpindexerCount = -1
        lastIntakeRunning = null
    }

    //stuff that copilot told me to add so "legacy wrappers still work" i didnt really understand it
    fun setLights(mode: Int) {
        setSpindexerLights(mode)
    }

    fun setDecorative() {
        prism.clearAllAnimations()
        prism.insertAndUpdateAnimation(
            GoBildaPrismDriver.LayerHeight.LAYER_0,
            PrismAnimations.Rainbow()
        )
        prism.insertAndUpdateAnimation(
            GoBildaPrismDriver.LayerHeight.LAYER_1,
            PrismAnimations.Sparkle()
        )
        lastMode = ActiveMode.NONE
    }

    fun setLights(index: Int, colorCode: Int) {
        if (index < 0 || index > 2) {
            return
        }

        val color: Color?
        if (colorCode == 0) {
            color = Color.TRANSPARENT
        } else if (colorCode == 1) {
            color = Color.PURPLE
        } else if (colorCode == 2) {
            color = Color.GREEN
        } else {
            return
        }

        val start = index * 4
        val end = min(start + 3, LAST_LED)
        prism.setSolidColor(start, end, color)
        prism.show()
        lastMode = ActiveMode.NONE
    }

    companion object {
        // if we ever add in another led strip we can change that stuff here
        private const val STRIP_LENGTH = 12
        private const val FIRST_LED = 0
        private val LAST_LED = STRIP_LENGTH - 1
    }
}