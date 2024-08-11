package org.firstinspires.ftc.teamcode.SubSystems;

public interface SubSystem {
    public void init();
    public void update();

}

// all sub systems should inherit this class that way we can be sure that there is a init and update function in the file