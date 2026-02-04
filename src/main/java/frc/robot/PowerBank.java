package frc.robot;

public class PowerBank {

    private PowerAccount[] accounts;
    private int size;
    private double maxPower = 600;

    public PowerBank(int numAccounts) {
        accounts = new PowerAccount[numAccounts];
        size = 0;
    }

    public PowerAccount openAccount(String name, int priority) {
        PowerAccount acc = new PowerAccount(name, 0, 0, priority);
        accounts[size] = acc;
        size++;
        return acc;
    }

    public double distributeMinRequest() {
        double remainingPower = maxPower;
        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            remainingPower -= acc.getMinRequest();
        }
        return remainingPower;
    }

    public void calculateAllowance(double remainingPower) {

        double totalReq = 0;
        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            totalReq += acc.getMaxRequest() - acc.getMinRequest();
        }

        if(totalReq <= remainingPower) {
            for(int i = 0; i < size; i++) {
                PowerAccount acc = accounts[i];
                acc.setAllowance(acc.getMaxRequest());
            }
            return;
        }

        double totalPriority = 0;
        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            totalPriority += acc.getMaxRequest() * acc.getPriority();
        }

        double totalPriorityInv = 0;
        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            totalPriorityInv += 1.0 / acc.getPriority();
        }

        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            double allowance = acc.getMaxRequest() - (totalReq - remainingPower)/(acc.getPriority() * totalPriorityInv);
        }
    }
}