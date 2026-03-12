package org.firstinspires.ftc.teamcode

import org.jetbrains.annotations.Range

enum class ArtefactType {
    GREEN, PURPLE, UNKNOWN
}

/**
 * Converts the fiducial ID from the obelisk apriltag into a list representing the motif
 *
 * @throws IllegalStateException If the id is not in the range 21-23
 */
fun @Range(from = 21, to = 23) Int.toArtefactList(): List<ArtefactType> = when (this) {
    21 -> listOf(GREEN, PURPLE, PURPLE)
    22 -> listOf(PURPLE, GREEN, PURPLE)
    23 -> listOf(PURPLE, PURPLE, GREEN)
    else -> error("Invalid fiducialId: $this")
}
