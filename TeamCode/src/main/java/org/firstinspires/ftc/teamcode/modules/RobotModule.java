package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotModule {
    public Data data;

    /**
     * This has no params and should be overridden by the extending class
     * There should not be any needed data as input and all data should be in the Data class.
     */
    public void run(){

    }

    /**
     * default init function, should not be overridden
     * @param data Needs to be passed on init loop (not object creation) The Created Data class in the init loop
     * @author GoldStar184
     */
    public final void init(Data data){
        this.data = data;
    }

    /**
     * Im hoping that this is not needed, but just in case it is...
     *
     * @param telemetry the telemetry object to add telemetry to.
     * @return the same telemetry object that was passed in. (with telemetry added)
     */
    Telemetry telemetry(Telemetry telemetry){
        return telemetry;
    }
}
