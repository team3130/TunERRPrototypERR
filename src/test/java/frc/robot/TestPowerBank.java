package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;


public class TestPowerBank {
        static PowerAccount hungry;
        static PowerAccount important;

    @BeforeAll
    public void initialize(){
        hungry = PowerBank.getInstance().openAccount("Hungry", 1);
        important = PowerBank.getInstance().openAccount("Important", 1000);
    }
    
    @Test
    public void inBudget() {

        hungry.setMinRequest(0);
        hungry.setMaxRequest(200);
        important.setMinRequest(100);
        important.setMaxRequest(100);

        PowerBank.getInstance().calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 100, "important failed");
        assertEquals(hungry.getAllowance(), 200, "hungry failed");
    }

    @Test
    public void limitHungry() {

        hungry.setMinRequest(0);
        hungry.setMaxRequest(400);
        important.setMinRequest(400);
        important.setMaxRequest(400);

        PowerBank.getInstance().calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 400, "important failed");
        assertEquals(hungry.getAllowance(), 200, "hungry failed");
    }

    @Test
        public void overBudget() {

        hungry.setMinRequest(400);
        hungry.setMaxRequest(400);
        important.setMinRequest(400);
        important.setMaxRequest(400);

        PowerBank.getInstance().calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 400);
        assertEquals(hungry.getAllowance(), 400);
    }

    @Test
        public void equals() {

        hungry.setMinRequest(400);
        hungry.setMaxRequest(400);
        important.setMinRequest(400);
        important.setMaxRequest(400);

        PowerBank.getInstance().calculate();

        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 400);
        assertEquals(hungry.getAllowance(), 400);
        }


}