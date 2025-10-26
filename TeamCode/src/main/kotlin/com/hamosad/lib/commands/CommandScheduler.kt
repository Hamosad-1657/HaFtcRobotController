package com.hamosad.lib.commands

object CommandScheduler {
    private val activeCommands: MutableList<Command> = mutableListOf()

    private var subsystems: MutableList<Subsystem> = mutableListOf()

    /** Register a subsystem to the scheduler. note that the subsystem is not wiped when reset is called. To wipe subsystems use wipeSubsystems */
    fun registerSubsystem(subsystem: Subsystem) {
        if (!subsystems.contains(subsystem)) subsystems.add(subsystem)
    }

    fun wipeSubsystems() {
        subsystems.clear()
    }


    /** Call after assigning default commands for subsystems and registering subsystems, and when robot is supposed to start. */
    fun initialize() {
        for (subsystem in subsystems) {
            if (subsystem.defaultCommand != null) {
                activeCommands.add(subsystem.defaultCommand!!)
            }
        }
    }

    /** Schedules one command. meant to be used while robot is operating. */
    fun scheduleCommand(command: Command) {
        if (activeCommands.contains(command)) return

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