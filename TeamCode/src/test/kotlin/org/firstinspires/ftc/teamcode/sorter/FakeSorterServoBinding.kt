package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.*
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.metro.SorterProviders

//TODO: Nici cea mai mica idee de ce nu pot pune clasa asta in test source set
@ContributesTo(OpModeScope::class, replaces = [SorterProviders::class])
@BindingContainer
object FakeSorterServoBinding {
    lateinit var sorterServo: FakeServo

    @Provides
    @Named("SorterServo")
    fun provideShooterServo(fakeServoProvider: Provider<FakeServo>): Servo {
        sorterServo = fakeServoProvider()
        return sorterServo
    }
}