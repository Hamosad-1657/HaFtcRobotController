package com.hamosad.lib.commands

import com.qualcomm.robotcore.hardware.HardwareMap

// TODO: Add telemetry
/** A singleton that handles commands, subsystem periodic, and telemetry sent by subsystems. */
object CommandScheduler {
    private val activeCommands: MutableList<Command> = mutableListOf()

    val subsystems: MutableList<Subsystem> = mutableListOf()

    private val triggers: MutableList<Trigger> = mutableListOf()

    /** Register a subsystem to the scheduler. note that the subsystem is not wiped when reset is called. To wipe subsystems use [wipeSubsystems] */
    fun registerSubsystem(subsystem: Subsystem) {
        if (!subsystems.contains(subsystem)) {
            subsystems.add(subsystem)
        }
    }

    /** Not intended to be used regularly. */
    fun wipeSubsystems() {
        subsystems.clear()
    }

    fun registerTrigger(trigger: Trigger) {
        triggers.add(trigger)
    }

    fun wipeTriggers() {
        triggers.clear()
    }

    /** Call after assigning default commands for subsystems and registering subsystems, and when robot is supposed to start. */
    fun initialize() {
        for (subsystem in subsystems) {
            if (subsystem.defaultCommand != null) {
                scheduleCommand(subsystem.defaultCommand!!)
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
        // Trigger handling
        for (trigger in triggers) {
            for (command in trigger.getCommandsToRun()) {
                if (command !in activeCommands) {
                    activeCommands.add(command)
                }
            }

            for (command in trigger.getCommandsToNotRun()) {
                if (command in activeCommands) {
                    activeCommands.remove(command)
                }
            }
        }

        // Subsystem handling
        for (subsystem in subsystems) {
            subsystem.periodic()
        }

        // Command handling
        val iterator = activeCommands.iterator()
            while (iterator.hasNext()) {
                val command = iterator.next()

                command.execute()
                if (command.isFinished()) {
                    command.onEnd(false)
                    iterator.remove()
                }
            }

        // Subsystem default command handling
        for (subsystem in subsystems) {
            if (!activeCommands.any { it.requirements.any { it == subsystem } }) {
                if (subsystem.defaultCommand != null) {
                    scheduleCommand(subsystem.defaultCommand!!)
                }
            }
        }

    }

    /** Wipes the schedulers memory of commands and triggers, and appropriately ends them. Use when transitioning from auto to teleop or anything similar. */
    fun reset() {
        wipeTriggers()
        for (command in activeCommands) {
            command.onEnd(true)
        }
        activeCommands.clear()
    }
}