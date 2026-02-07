package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class PowerBank {

    private final ArrayList<PowerAccount> accounts;
    private final double maxPower = 600;
    private double remainingPower;

    public PowerBank() {
        accounts = new ArrayList<PowerAccount>();
    }

    public PowerAccount openAccount(String name, int priority) {
        PowerAccount acc = new PowerAccount(name, 0, 0, priority);
        accounts.add(acc);
        return acc;
    }


    public void calculateAllowance(List<PowerAccount> accounts) {
        ArrayList<PowerAccount> nonzeroAccounts = new ArrayList<PowerAccount>();
        remainingPower = maxPower;


        for(PowerAccount acc: accounts) {
            acc.setAllowance(acc.getMinRequest());
            remainingPower -= acc.getMinRequest();
            if(acc.getMaxRequest() - acc.getMinRequest() > 0) {
                nonzeroAccounts.add(acc);
            }
        }

        double totalOverflow = 0;
        for(PowerAccount acc: nonzeroAccounts) {
            totalOverflow += acc.getMaxRequest() - acc.getMinRequest();
        }

        if(totalOverflow <= remainingPower) {
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
            double extraAllowance = acc.getMaxRequest() - acc.getMinRequest() - (totalOverflow - remainingPower)/(acc.getPriority() * totalPriorityInv);
            if(extraAllowance < 0) {
                nonzeroAccounts.remove(acc);
                calculateAllowance(nonzeroAccounts);
                return;
            }
            acc.setAllowance(extraAllowance + acc.getMinRequest());
        }
    }

    public List<PowerAccount> getAccounts() {
        return accounts;
    }
}