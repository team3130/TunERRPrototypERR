package frc.robot;

public class PowerAccount {

    private String name;
    private double request;
    private double allowance;
    private int priority;

    public PowerAccount(String name, double request, double allowance, int priority) {
        this.name = name;
        this.request = request;
        this.allowance = allowance;
        this.priority = priority;
    }

    public String getName() { return name; }
    public double getRequest() { return request; }
    public double getAllowance() { return allowance; }
    public int getPriority() { return priority; }

    public void setAllowance(double allowance) { this.allowance = allowance; }
    public void setRequest(double request) { this.request = request; }
}
