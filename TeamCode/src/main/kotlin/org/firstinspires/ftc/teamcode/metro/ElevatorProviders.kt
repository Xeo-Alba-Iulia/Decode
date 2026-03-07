package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.elevator.Elevator

@ContributesTo(HardwareScope::class)
interface ElevatorProviders {
    val elevator: Elevator

    @Provides
    @Named("elevator")
    fun provideElevatorMotor(map: HardwareMap): DcMotorEx = map.getCast("elevator")
}
