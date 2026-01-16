package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides

@ContributesTo(OpModeScope::class)
interface ShooterProviders {
    @Provides
    @Named("shooterHoodServo")
    fun provideShooterHoodServo(map: HardwareMap): Servo = map.servo["hood"].apply {
        direction = Servo.Direction.REVERSE
        scaleRange(0.3, 0.85)
    }

    @Provides
    @Named("shooterRotationServo")
    fun provideShooterRotationServo(map: HardwareMap): Servo = map.servo["rotation"].apply {
        direction = Servo.Direction.REVERSE
    }

    @Provides
    @Named("shooterMotor")
    fun provideShooterMotor(map: HardwareMap): DcMotorEx = map.dcMotor["shooter"] as DcMotorEx

    @Provides
    @Named("shooterEncoder")
    fun provideShooterEncoder(map: HardwareMap): DcMotorEx = (map.dcMotor["encoder"] as DcMotorEx).apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
}