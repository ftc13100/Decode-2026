package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.FeedbackElement

object Shooter {
}

class FullPowerFeedback : FeedbackElement {
    override fun calculate(error: KineticState): Double {
        return 1.0
    }
}

controlSystem {
    feedback(FullPowerFeedback())
}