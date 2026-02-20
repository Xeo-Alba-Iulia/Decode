package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.elevator.Elevator

@ContributesTo(OpModeScope::class)
interface ElevatorProviders {
    val elevator: Elevator

    @Provides
    fun provideServos(map: HardwareMap): List<Servo> = listOf(
        map.servo["lift left"],
        map.servo["lift right"].apply { direction = Servo.Direction.REVERSE },
    )
}