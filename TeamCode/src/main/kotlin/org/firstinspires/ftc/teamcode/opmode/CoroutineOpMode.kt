package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.createGraphFactory
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

abstract class CoroutineOpMode(protected val observers: MutableList<OpModeObserver> = mutableListOf()) : OpMode() {
    val opModeGraph by lazy { createGraphFactory<OpModeGraph.Factory>().create(this) }
    protected val opModeScope = opModeGraph.opModeScope

    override fun start() {
        for (observer in observers) {
            opModeScope.launch {
                observer.onStart(this@CoroutineOpMode)
            }
        }
    }

    override fun stop() {
        for (observer in observers.asReversed()) {
            opModeScope.launch {
                observer.onStop(this@CoroutineOpMode)
            }
        }
        opModeScope.cancel("OpMode stopped")
    }
}