package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provider
import org.firstinspires.ftc.teamcode.pedropathing.TuningOpModeKey

@ContributesTo(HardwareScope::class)
interface TuningInjection {
    val tuningOpModesMap: Map<TuningOpModeKey, Provider<OpMode>>
}