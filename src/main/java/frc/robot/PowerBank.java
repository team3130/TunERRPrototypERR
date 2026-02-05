package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class PowerBank {

    private ArrayList<PowerAccount> accounts;
    private double maxPower = 600;

    public PowerBank() {
        accounts = new ArrayList<PowerAccount>();
    }

    public PowerAccount openAccount(String name, int priority) {
        PowerAccount acc = new PowerAccount(name, 0, 0, priority);
        accounts.add(acc);
        return acc;
    }

    public double distributeMinRequest() {
        double remainingPower = maxPower;
        for(PowerAccount acc: accounts) {
            remainingPower -= acc.getMinRequest();
        }
        return remainingPower;
    }

    public void calculateAllowance(double remainingPower) {

        double totalReq = 0;
        for(PowerAccount acc: accounts) {
            totalReq += acc.getMaxRequest() - acc.getMinRequest();
        }

        if(totalReq <= remainingPower) {
            for(PowerAccount acc: accounts) {
                acc.setAllowance(acc.getMaxRequest());
            }
            return;
        }

        double totalPriority = 0;
        for(PowerAccount acc: accounts) {
            totalPriority += acc.getMaxRequest() * acc.getPriority();
        }

        double totalPriorityInv = 0;
        for(PowerAccount acc: accounts) {
            totalPriorityInv += 1.0 / acc.getPriority();
        }

        for(PowerAccount acc: accounts) {
            double allowance = acc.getMaxRequest() - (totalReq - remainingPower)/(acc.getPriority() * totalPriorityInv);
            acc.setAllowance(allowance);
        }
    }
}