package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.asContribution
import dev.zacsweers.metro.createGraphFactory
import kotlinx.coroutines.cancel
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.CoroutineProviders
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

abstract class CoroutineOpMode : OpMode() {
    val opModeGraph = createGraphFactory<OpModeGraph.Factory>().create(this)
    protected val opModeScope = opModeGraph.asContribution<CoroutineProviders>().opModeScope
    private val observerList = mutableListOf<OpModeObserver>()

    protected fun addObserver(observer: OpModeObserver) {
        observerList += observer
    }
    protected fun addObservers(vararg observers: OpModeObserver) {
        observerList += observers
    }

    protected fun removeObserver(observer: OpModeObserver) {
        observerList -= observer
    }
    protected fun removeObservers(vararg observers: OpModeObserver) {
        observerList -= observers.toSet()
    }

    override fun start() {
        for (observer in observerList) {
            opModeScope.launch {
                observer.onStart(this@CoroutineOpMode)
            }
        }
    }

    override fun loop() {}

    override fun stop() {
        for (observer in observerList.asReversed()) {
            opModeScope.launch {
                observer.onStop(this@CoroutineOpMode)
            }
        }
        opModeScope.cancel("OpMode stopped")
    }
}