package frc.robot.common;

public enum TofType {
    CTRE("CTRE"), 
    PWF("PWF"),
    Laser("Laser");

    private String name;
    
    TofType(String name) {
        this.name = name;
    }

    public String getName() {
        return this.name;
    }
}
