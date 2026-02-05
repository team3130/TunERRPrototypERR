package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class PowerBank {

    private final ArrayList<PowerAccount> accounts;
    private final double maxPower = 600;
    private final double deadband = -0.01;

    public PowerBank() {
        accounts = new ArrayList<PowerAccount>();
    }

    public PowerAccount openAccount(String name, int priority) {
        PowerAccount acc = new PowerAccount(name, 0, 0, priority);
        accounts.add(acc);
        return acc;
    }

    public ArrayList<PowerAccount> findNonzeroOverflow() {
        ArrayList<PowerAccount> nonzeroAccounts = new ArrayList<PowerAccount>();
        for(PowerAccount acc: accounts) {
            acc.setAllowance(acc.getMinRequest());
            if(acc.getMaxRequest() - acc.getMinRequest() > 0) {
                nonzeroAccounts.add(acc);
            }
        }
        return nonzeroAccounts;
    }

    public void calculateAllowance(List<PowerAccount> nonzeroAccounts) {
        double remainingPower = maxPower;
        for(PowerAccount acc: nonzeroAccounts) {
            remainingPower -= acc.getMinRequest();
        }

        double totalReq = 0;
        for(PowerAccount acc: nonzeroAccounts) {
            totalReq += acc.getMaxRequest() - acc.getMinRequest();
        }

        if(totalReq <= remainingPower) {
            for(PowerAccount acc: nonzeroAccounts) {
                acc.setAllowance(acc.getMaxRequest());
            }
            return;
        }

        double totalPriorityInv = 0;
        for(PowerAccount acc: nonzeroAccounts) {
            totalPriorityInv += 1.0 / acc.getPriority();
        }

        for(PowerAccount acc: nonzeroAccounts) {
            double allowance = acc.getMaxRequest() - acc.getMinRequest() - (totalReq - remainingPower)/(acc.getPriority() * totalPriorityInv);
            if(allowance < deadband) {
                nonzeroAccounts.remove(acc);
                calculateAllowance(nonzeroAccounts);
                return;
            }
            acc.setAllowance(allowance + acc.getMinRequest());
        }
    }
}