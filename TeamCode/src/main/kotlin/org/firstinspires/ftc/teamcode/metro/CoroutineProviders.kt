package org.firstinspires.ftc.teamcode.metro

import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.shareIn

@ContributesTo(OpModeScope::class)
interface CoroutineProviders {
    val opModeScope: CoroutineScope
    val tickFlow: SharedFlow<Unit>

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideOpModeScope(): CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideTickFlow(scope: CoroutineScope): SharedFlow<Unit> =
        flow {
            while (true) {
                emit(Unit)
                delay(20L)
            }
        }.shareIn(scope, SharingStarted.Eagerly, replay = 0)
}