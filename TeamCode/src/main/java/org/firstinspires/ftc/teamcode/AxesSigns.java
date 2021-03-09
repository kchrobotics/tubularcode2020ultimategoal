package org.firstinspires.ftc.teamcode;

public enum AxesSigns {
    PPP(0),
    PPN(1),
    PNP(2),
    PNN(3),
    NPP(4),
    NPN(5),
    NNP(6),
    NNN(7);
    
    public final int bVal;

    private AxesSigns(int bVal2) {
        this.bVal = bVal2;
    }
}
