package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.metro.SorterProviders

@ContributesTo(OpModeScope::class, replaces = [SorterProviders::class])
interface FakeSorterServoBinding {
    @Named("sorterServo")
    val sorterServo: Servo

    @Provides
    @SingleIn(OpModeScope::class)
    @Named("sorterServo")
    fun provideShooterServo(servo: FakeServo): Servo = servo
}