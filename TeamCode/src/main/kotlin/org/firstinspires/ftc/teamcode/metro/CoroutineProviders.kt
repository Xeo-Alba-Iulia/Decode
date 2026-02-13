package org.firstinspires.ftc.teamcode.metro

import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob

@ContributesTo(OpModeScope::class)
interface CoroutineProviders {
    val opModeScope: CoroutineScope

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideOpModeScope(): CoroutineScope = CoroutineScope(SupervisorJob())
}
