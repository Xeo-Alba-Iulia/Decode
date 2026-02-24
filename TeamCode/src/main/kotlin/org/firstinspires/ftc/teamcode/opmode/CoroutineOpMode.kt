package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.createGraphFactory
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

abstract class CoroutineOpMode(
    protected val observers: MutableList<OpModeObserver> = mutableListOf(),
    isAuto: Boolean? = null
) : OpMode() {
    val isAuto = isAuto ?: (this::class.java.annotations.count { it is Autonomous } >= 1)
    val opModeGraph = createGraphFactory<OpModeGraph.Factory>().create(this, this.isAuto)
    protected val opModeScope = opModeGraph.opModeScope

    override fun start() {
        for (observer in observers) {
            opModeScope.launch {
                observer.onStart(this@CoroutineOpMode)
            }
        }
    }

    override fun loop() {}

    override fun stop() {
        for (observer in observers.asReversed()) {
            opModeScope.launch {
                observer.onStop(this@CoroutineOpMode)
            }
        }
        opModeScope.cancel("OpMode stopped")
    }
}