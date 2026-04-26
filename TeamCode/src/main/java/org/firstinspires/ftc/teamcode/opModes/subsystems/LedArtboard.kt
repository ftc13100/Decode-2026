package org.firstinspires.ftc.teamcode.opModes.subsystems

import org.firstinspires.ftc.teamcode.opModes.subsystems.Prism.*
import kotlin.math.max
import kotlin.math.min

class LedArtboard(private val prism: GoBildaPrismDriver) {

    // coach wants this conceptually to just be on/off (intake running or not)
    private enum class ActiveMode {
        OFF,
        ON
    }

    // cache so we only write to the led driver when something actually changed
    private var lastMode = ActiveMode.OFF
    private var lastBallCount = -1
    private var lastIntakeRunning: Boolean? = null
    private var lastArtboard = -1

    init {
        this.prism.setStripLength(STRIP_LENGTH)

        // artboards are assumed to be preconfigured on the prism:
        //
        // artboard 0 - 0 balls, intake stopped - solid red
        // artboard 1 - 0 balls, intake running - blinking red
        // artboard 2 - 1 ball,  intake stopped - solid blue
        // artboard 3 - 1 ball,  intake running - blinking blue
        // artboard 4 - 2 balls, intake stopped - solid yellow
        // artboard 5 - 2 balls, intake running - blinking yellow
        // artboard 6 - 3 balls, intake stopped - solid green
        // artboard 7 - 3 balls, intake running - blinking green
    }

    /**
     * unified led state:
     *
     * - ballcount picks the color (0=red, 1=blue, 2=yellow, 3=green)
     * - intakerunning picks solid vs blinking
     *
     * only writes to the prism if something actually changed
     */
    fun setIntakeAndSpindexerLights(ballCount: Int, intakeRunning: Boolean) {
        val clamped = max(0, min(ballCount, 3))

        // skip if nothing changed
        if (clamped == lastBallCount && lastIntakeRunning == intakeRunning) return

        // compute artboard index
        val artboardIndex = clamped * 2 + if (intakeRunning) 1 else 0

        // avoid redundant writes
        if (artboardIndex != lastArtboard) {
            val artboard = GoBildaPrismDriver.Artboard.values()[artboardIndex]
            prism.loadAnimationsFromArtboard(artboard)
            lastArtboard = artboardIndex
        }

        lastBallCount = clamped
        lastIntakeRunning = intakeRunning
        lastMode = if (intakeRunning) ActiveMode.ON else ActiveMode.OFF
    }

    /**
     * legacy wrapper:
     * spindexer lights assume intake is stopped (solid)
     */
    fun setSpindexerLights(ballCount: Int) {
        setIntakeAndSpindexerLights(ballCount, false)
    }

    /**
     * legacy wrapper:
     * intake lights use last known ballcount
     */
    fun setIntakeLights(running: Boolean) {
        val count = if (lastBallCount >= 0) lastBallCount else 0
        setIntakeAndSpindexerLights(count, running)
    }

    /**
     * clear = treat as 0 balls, intake stopped
     */
    fun clear() {
        setIntakeAndSpindexerLights(0, false)
    }

    /**
     * legacy wrapper:
     * interpret mode as ballcount, intake stopped
     */
    fun setLights(mode: Int) {
        setSpindexerLights(mode)
    }

    /**
     * decorative mode uses animations directly
     * this is intentionally separate from artboard logic
     */
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

        lastMode = ActiveMode.OFF
        lastBallCount = -1
        lastIntakeRunning = null
        lastArtboard = -1
    }

    /**
     * legacy per-section api:
     * intentionally no-op to avoid slow i2c pixel writes
     */
    fun setLights(index: Int, @Suppress("UNUSED_PARAMETER") colorCode: Int) {
        if (index !in 0..2) return
        // no-op
    }

    companion object {
        //if we ever add more leds we can change that stuff here
        private const val STRIP_LENGTH = 12
    }
}
