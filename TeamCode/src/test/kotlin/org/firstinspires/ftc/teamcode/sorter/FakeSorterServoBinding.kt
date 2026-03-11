package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.*
import org.firstinspires.ftc.teamcode.FakeCRServo
import org.firstinspires.ftc.teamcode.FakeServo
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

    @Binds
    @Named("transferServo")
    private val FakeCRServo.bind: CRServo get() = this
}