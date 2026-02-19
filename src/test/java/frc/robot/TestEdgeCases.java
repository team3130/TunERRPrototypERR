package frc.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;


public class TestEdgeCases {
        static PowerAccount one;
        static PowerAccount two;
        static PowerAccount three;
        static PowerAccount four;

    @BeforeAll
    public static void initialize(){
        one = PowerBank.getTestInstance().openAccount("one", 1);
        two = PowerBank.getTestInstance().openAccount("two", 10);
        three = PowerBank.getTestInstance().openAccount("three", 10);
        four = PowerBank.getTestInstance().openAccount("four", 10);
    }

    @Test
    public void martyr() {

        one.setMinRequest(0);
        one.setMaxRequest(10000);
        two.setMinRequest(0);
        two.setMaxRequest(10000);
        three.setMinRequest(0);
        three.setMaxRequest(10000);
        four.setMinRequest(0);
        four.setMaxRequest(10000);

        PowerBank.getTestInstance().calculate();

        System.out.println("one Allowance: " + one.getAllowance());
        System.out.println("two Allowance: " + two.getAllowance());
        System.out.println("three Allowance: " + three.getAllowance());
        System.out.println("four Allowance: " + four.getAllowance());
    }
}