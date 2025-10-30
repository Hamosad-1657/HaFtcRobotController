package com.hamosad.lib.opModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class TimedRobotOpMode: OpMode() {
    /** Called once when OpMode init is pressed. */
    abstract fun disabledInit()

    // Called when init is pressed
    /** Do not override this. */
    final override fun init() {
        disabledInit()
    }

    /** Called repeatedly after init is pressed. */
    abstract fun disabledPeriodic()

    // Called repeatedly after init is pressed
    /** Do not override this. */
    final override fun init_loop() {
        super.init_loop()
        disabledPeriodic()
    }

    /** Called once after start is pressed. */
    abstract fun startInit()

    // Called when start is pressed
    /** Do not override this. */
    final override fun start() {
        super.start()
        startInit()
    }


    /** Called repeatedly after start is pressed. */
    abstract fun startPeriodic()

    // Called repeatedly after start is pressed
    /** Do not override this. */
    final override fun loop() {
        startPeriodic()
    }

    /** Called once after stop is pressed. */
    abstract fun onEnd()

    // Called when stop is pressed
    /** Do not override this. */
    final override fun stop() {
        onEnd()
    }
}