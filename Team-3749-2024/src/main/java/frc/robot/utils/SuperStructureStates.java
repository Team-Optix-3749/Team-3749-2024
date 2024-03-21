package frc.robot.utils;

import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;

public enum SuperStructureStates{
    STOW(WristStates.STOW, ArmStates.STOW),
    GROUND_INTAKE(WristStates.ALMOST_DEPLOYED, ArmStates.STOW),
    AMP(WristStates.FULL_DEPLOYED, ArmStates.AMP),
    SHOOT(WristStates.FULL_DEPLOYED, ArmStates.SHOOT),
    CLIMB(WristStates.STOW, ArmStates.CLIMB),
    CLIMBDOWN(WristStates.STOW, ArmStates.STOW),
    SUBWOOFER(WristStates.FULL_DEPLOYED, ArmStates.SUBWOOFER),
    PODIUM(WristStates.FULL_DEPLOYED,ArmStates.PODIUM),
    AIMBOT(WristStates.STOW, ArmStates.AIMBOT),
    SOURCE(WristStates.STOW, ArmStates.SOURCE),
    RESET(WristStates.STOW, ArmStates.STOW);

    
    public WristStates wristState;
    public ArmStates armState;

    private SuperStructureStates(WristStates wristState, ArmStates armState){
        this.wristState = wristState;
        this.armState = armState;
    }
}