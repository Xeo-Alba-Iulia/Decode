package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.*
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
    fun provideOpModeScope(): CoroutineScope = CoroutineScope(SupervisorJob())

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideTickFlow(scope: CoroutineScope, map: HardwareMap): SharedFlow<Unit> =
        flow {
            val hubs = map.getAll(LynxModule::class.java)
            for (hub in hubs) {
                hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            }
            while (true) {
                emit(Unit)
                delay(25L)
            }
        }.shareIn(scope + Dispatchers.IO, SharingStarted.Eagerly, replay = 0)
}
