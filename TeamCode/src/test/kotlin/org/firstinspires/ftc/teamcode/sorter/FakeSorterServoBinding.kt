package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.BindingContainer
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provider
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.metro.SorterProviders

//TODO: Nici cea mai mica idee de ce nu pot pune clasa asta in test source set
@ContributesTo(OpModeScope::class, replaces = [SorterProviders::class])
@BindingContainer
object FakeSorterServoBinding {
    lateinit var fakeServo: FakeServo

    @Provides
    @Named("SorterServo")
    fun provideShooterServo(fakeServoProvider: Provider<FakeServo>): Servo {
        if (!::fakeServo.isInitialized) {
            fakeServo = fakeServoProvider()
        }
        return fakeServo
    }
}