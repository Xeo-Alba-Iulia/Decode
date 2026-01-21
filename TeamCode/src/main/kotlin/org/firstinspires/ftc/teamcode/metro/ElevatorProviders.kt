package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.elevator.Elevator

@ContributesTo(OpModeScope::class)
interface ElevatorProviders {
    val elevator: Elevator

    @Provides
    @Suppress("UNCHECKED_CAST")
    fun provideElevatorMotors(map: HardwareMap): List<DcMotorEx> =
        listOf(
            map.dcMotor["lift left"].apply { direction = DcMotorSimple.Direction.REVERSE },
            map.dcMotor["lift right"],
        ) as List<DcMotorEx>
}