package org.firstinspires.ftc.teamcode.metro

import dev.zacsweers.metro.ContributesTo
import org.firstinspires.ftc.teamcode.pedropathing.Tuning

@ContributesTo(OpModeScope::class)
interface TuningInjection {
    fun inject(opMode: Tuning)
}