package com.hamosad.lib.commands

class CommandScheduler(val subsystems: List<Subsystem>) {
    val activeCommands: MutableList<Command> = mutableListOf()


    /** Call after assigning default commands for subsystems */
    fun initialize() {
        for (subsystem in subsystems) {
            if (subsystem.defaultCommand != null) {
                scheduleCommand(subsystem.defaultCommand!!)
            }
        }
    }

    /** Schedules one command. meant to be used while robot is operating. */
    fun scheduleCommand(command: Command) {
        val iterator = activeCommands.iterator()

        while (iterator.hasNext()) {
            val activeCommand = iterator.next()

            if (activeCommand.requirements.any {it in command.requirements}) {
                activeCommand.onEnd(true)
                iterator.remove()
            }
        }
        command.initialize()
        activeCommands.add(command)
    }

    /** Core loop function. */
    fun execute() {
        for (subsystem in subsystems) {
            subsystem.periodic()
        }

        val iterator = activeCommands.iterator()
            while (iterator.hasNext()) {
                val command = iterator.next()

                command.execute()
                if (command.isFinished()) {
                    command.onEnd(false)
                    iterator.remove()
                }
            }

        for (subsystem in subsystems) {
            if (!activeCommands.any { it.requirements.any { it == subsystem } }) {
                if (subsystem.defaultCommand != null) {
                    scheduleCommand(subsystem.defaultCommand!!)
                }
            }
        }

    }

    /** Wipes the schedulers memory of commands, and appropriately ends them. Use when transitioning from auto to teleop or anything similar. */
    fun reset() {
        for (command in activeCommands) {
            command.onEnd(true)
        }
        activeCommands.clear()
    }
}