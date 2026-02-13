package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestPowerBank {
    @Test
    public void simple() {
        PowerBank powerBank = new PowerBank();

        PowerAccount hungry = powerBank.openAccount("Hungry", 1);
        PowerAccount walking = powerBank.openAccount("Walking", 50);
        PowerAccount important = powerBank.openAccount("Important", 1000);

        hungry.setMinRequest(20);
        hungry.setMaxRequest(300);
        walking.setMinRequest(20);
        walking.setMaxRequest(100);
        important.setMinRequest(30);
        important.setMaxRequest(30);

        powerBank.calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Walking Allowance: " + walking.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 30);
    }

    @Test
    public void overPowerTest() {
        PowerBank powerBank = new PowerBank();

        PowerAccount hungry = powerBank.openAccount("Hungry", 1);
        PowerAccount walking = powerBank.openAccount("Walking", 50);
        PowerAccount important = powerBank.openAccount("Important", 1000);

        hungry.setMinRequest(20);
        hungry.setMaxRequest(700);
        walking.setMinRequest(20);
        walking.setMaxRequest(100);
        important.setMinRequest(30);
        important.setMaxRequest(30);

        powerBank.calculate();

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
        hungry.setMaxRequest(700);
        walking.setMinRequest(20);
        walking.setMaxRequest(100);
        important.setMinRequest(30);
        important.setMaxRequest(30.3);

        powerBank.calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Walking Allowance: " + walking.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());
    }
}
