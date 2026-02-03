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

    public void calculateAllowance() {

        double totalReq = 0;
        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            totalReq += acc.getMaxRequest();
        }

        if(totalReq == 0) {
            for(int i = 0; i < size; i++) {
                accounts[i].setAllowance(0);
            }
            return;
        }

        if(totalReq <= maxPower) {
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

        for(int i = 0; i < size; i++) {
            PowerAccount acc = accounts[i];
            double weight = acc.getMaxRequest() * acc.getPriority();
            double allowance = (weight / totalPriority) * maxPower;
            acc.setAllowance(allowance);
        }
    }
}