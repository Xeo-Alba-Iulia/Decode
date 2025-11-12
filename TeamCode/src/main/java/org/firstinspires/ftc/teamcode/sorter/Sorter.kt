package org.firstinspires.ftc.teamcode.sorter

interface Sorter {
    /**
     * Intakes an artefact
     *
     * @return `true` if there is space for an artefact, otherwise `false`
     */
    fun intake(type: ArtefactType): Boolean

    /**
     * Gets an artefact ready to be picked up by the shooter
     *
     * @param type Specifies the type of artefact. If null, it picks up the first available artefact.
     * @return `true` if the shooter has a ball of the desired type (or any ball if [type] is null), `false` otherwise.
     */
    operator fun get(type: ArtefactType? = null): Boolean
}