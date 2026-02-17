package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;


public class TestPowerBank {
        static PowerAccount hungry;
        static PowerAccount important;
        static PowerAccount cain;
        static PowerAccount abel;

    @BeforeAll
    public static void initialize(){
        hungry = PowerBank.getInstance().openAccount("Hungry", 1);
        important = PowerBank.getInstance().openAccount("Important", 1000);
        cain = PowerBank.getInstance().openAccount("cain", 10);
        abel = PowerBank.getInstance().openAccount("abel", 10);
    }

    @Test
    public void inBudget() {

        hungry.setMinRequest(0);
        hungry.setMaxRequest(200);
        important.setMinRequest(100);
        important.setMaxRequest(100);
        cain.setMinRequest(0);
        cain.setMaxRequest(0);
        abel.setMinRequest(0);
        abel.setMaxRequest(0);

        PowerBank.getInstance().calculate();

        System.out.println("cain Allowance: " + cain.getAllowance());
        System.out.println("abel Allowance: " + abel.getAllowance());
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
        cain.setMinRequest(0);
        cain.setMaxRequest(0);
        abel.setMinRequest(0);
        abel.setMaxRequest(0);

        PowerBank.getInstance().calculate();

        System.out.println("cain Allowance: " + cain.getAllowance());
        System.out.println("abel Allowance: " + abel.getAllowance());
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
        cain.setMinRequest(0);
        cain.setMaxRequest(0);
        abel.setMinRequest(0);
        abel.setMaxRequest(0);

        PowerBank.getInstance().calculate();

        System.out.println("cain Allowance: " + cain.getAllowance());
        System.out.println("abel Allowance: " + abel.getAllowance());
        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(important.getAllowance(), 400);
        assertEquals(hungry.getAllowance(), 400);
    }

    @Test
        public void equals() {
        
        hungry.setMinRequest(0);
        hungry.setMaxRequest(0);
        important.setMinRequest(0);
        important.setMaxRequest(0);
        cain.setMinRequest(0);
        cain.setMaxRequest(400);
        abel.setMinRequest(0);
        abel.setMaxRequest(400);

        PowerBank.getInstance().calculate();

        System.out.println("cain Allowance: " + cain.getAllowance());
        System.out.println("abel Allowance: " + abel.getAllowance());
        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());

        assertEquals(cain.getAllowance(), 300);
        assertEquals(abel.getAllowance(), 300);
        }

    @Test
        public void pullAllTheStops() {

        hungry.setMinRequest(0);
        hungry.setMaxRequest(800);
        important.setMinRequest(100);
        important.setMaxRequest(100);
        cain.setMinRequest(30);
        cain.setMaxRequest(100);
        abel.setMinRequest(30);
        abel.setMaxRequest(100);

        PowerBank.getInstance().calculate();

        System.out.println("cain Allowance: " + cain.getAllowance());
        System.out.println("abel Allowance: " + abel.getAllowance());
        System.out.println("Hungry Allowance: " + hungry.getAllowance());
        System.out.println("Important Allowance: " + important.getAllowance());
        }
}