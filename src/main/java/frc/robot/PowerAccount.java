package frc.robot;

public class PowerAccount {

    private String name;
    private double minRequest;
    private double maxRequest;
    private double allowance;
    private int priority;

    public PowerAccount(String name, double minRequest, double maxRequest, double allowance, int priority) {
        this.name = name;
        this.minRequest = minRequest;
        this.maxRequest = maxRequest;
        this.allowance = allowance;
        this.priority = priority;
    }

    public String getName() { return name; }
    public double getMinRequest() { return minRequest; }
    public double getMaxRequest() { return maxRequest;}
    public double getAllowance() { return allowance; }
    public int getPriority() { return priority; }

    public void setAllowance(double allowance) { this.allowance = allowance; }
    public void setMinRequest(double request) { this.minRequest = request; }
    public void setMaxequest(double request) { this.maxRequest = request; }
}
