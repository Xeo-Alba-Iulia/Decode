package org.firstinspires.ftc.teamcode.metro

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.DependencyGraph
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.Shooter

@DependencyGraph(OpModeScope::class)
interface OpModeGraph {
    val opMode: OpMode
    val shooter: Shooter

    @Provides
    fun provideHardwareMap(opMode: OpMode): HardwareMap = opMode.hardwareMap

    @Provides
    @Named("StandardTelemetry")
    fun provideClassicTelemetry(opMode: OpMode): Telemetry = opMode.telemetry

    @Provides
    fun provideTelemetry(@Named("StandardTelemetry") telemetry: Telemetry): Telemetry =
        MultipleTelemetry(
            FtcDashboard.getInstance().telemetry,
            telemetry
        )

    @DependencyGraph.Factory
    fun interface Factory {
        fun create(@Provides opMode: OpMode): OpModeGraph
    }
}