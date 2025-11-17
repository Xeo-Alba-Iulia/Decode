package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.BindingContainer
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides

@ContributesTo(OpModeScope::class)
@BindingContainer
class SorterProviders {
    @Provides @Named("SorterServo") fun provideServo(map: HardwareMap): Servo = map.servo["sorter"]
}