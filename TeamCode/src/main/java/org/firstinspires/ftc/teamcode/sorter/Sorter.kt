package org.firstinspires.ftc.teamcode.sorter

sealed interface Sorter {
    /**
     * Prepares the sorter to receive a ball by preselecting a free slot
     *
     * Conforming implementations will also call this in [intake] as long as the sorter isn't full,
     * and in [shoot], when the sorter becomes empty
     *
     * As such, if you always empty the sorter when you start shooting, you never need this method
     *
     * @throws IllegalStateException If the sorter is full
     *
     * @see isFull
     * @see isEmpty
     */
    fun prepareIntake()

    /**
     * Sets the type of artefact that was put in the intake.
     *
     * Implementations of the class must call [prepareIntake] if the sorter isn't full
     *
     * @throws IllegalStateException If the sorter wasn't prepared for an intake.
     * If you encounter this you probably forgot to call [prepareIntake] at the beginning of your
     * [OpMode][com.qualcomm.robotcore.eventloop.opmode.OpMode.start]
     * @see [isFull]
     */
    fun intake(type: ArtefactType)

    /**
     * Supplies an artefact to be picked up by the shooter
     *
     * Implementations must call [prepareIntake] if following a call to this method the shooter becomes empty
     *
     * @see isEmpty
     */
    fun shoot(type: ArtefactType? = null): Boolean

    /**
     * Number of artefacts currently in the sorter
     */
    val size: Int

    val isEmpty get() = size == 0
    val isFull get() = size == 3
}