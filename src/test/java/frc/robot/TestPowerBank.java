package frc.robot;

import org.junit.jupiter.api.Test;

public class TestPowerBank {
    @Test
    public void firstTest() {
        PowerBank powerBank = new PowerBank();

        PowerAccount hungry = powerBank.openAccount("Hungry", 1);
        PowerAccount walking = powerBank.openAccount("Walking", 50);
        PowerAccount important = powerBank.openAccount("Important", 1000);

        hungry.setMinRequest(20);
        hungry.setMaxequest(300);
        walking.setMinRequest(20);
        walking.setMaxequest(100);
        important.setMinRequest(30);
        important.setMaxequest(30);

        powerBank.calculateAllowance(powerBank.distributeMinRequest());

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Walking Allowance: " + walking.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());
    }

    @Test
    public void overPowerTest() {
        PowerBank powerBank = new PowerBank();

        PowerAccount hungry = powerBank.openAccount("Hungry", 1);
        PowerAccount walking = powerBank.openAccount("Walking", 50);
        PowerAccount important = powerBank.openAccount("Important", 1000);

        hungry.setMinRequest(20);
        hungry.setMaxequest(700);
        walking.setMinRequest(20);
        walking.setMaxequest(100);
        important.setMinRequest(30);
        important.setMaxequest(30);

        powerBank.calculateAllowance(powerBank.distributeMinRequest());

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Walking Allowance: " + walking.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());
    }

    @Test
    public void overPowerTest2() {
        PowerBank powerBank = new PowerBank();

        PowerAccount hungry = powerBank.openAccount("Hungry", 1);
        PowerAccount walking = powerBank.openAccount("Walking", 50);
        PowerAccount important = powerBank.openAccount("Important", 1000);

        hungry.setMinRequest(20);
        hungry.setMaxequest(700);
        walking.setMinRequest(20);
        walking.setMaxequest(100);
        important.setMinRequest(30);
        important.setMaxequest(30.1);

        powerBank.calculateAllowance(powerBank.distributeMinRequest());

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Walking Allowance: " + walking.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());
    }
}
