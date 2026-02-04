package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Binds
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides

@ContributesTo(OpModeScope::class)
interface ShooterProviders {
    val limelight: Limelight3A

    @Provides
    @Named("shooterHoodServo")
    fun provideShooterHoodServo(map: HardwareMap): Servo = map.servo["hood"].apply {
        direction = Servo.Direction.REVERSE
//        scaleRange(0.0, 0.9)
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
    @Named("shooterSecondary")
    fun provideSecondShooterMotor(map: HardwareMap): DcMotorEx = map.getCast("shooter1")

    @Provides
    @Named("shooter")
    fun provideShooterMotors(
        @Named("shooterMotor") motor1: DcMotorEx,
        @Named("shooterSecondary") motor2: DcMotorEx,
    ): List<DcMotorEx> = listOf(motor1, motor2)

    @Provides
    fun provideLimelight(map: HardwareMap): Limelight3A = map.getAll(Limelight3A::class.java).single()

    @Binds
    @Named("shooterEncoder")
    val @receiver:Named("shooterMotor") DcMotorEx.bindEncoder: DcMotorEx
}