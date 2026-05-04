within ModelicaTest.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines;
model IMC_Initialize "Steady-state initialization of induction machine with squirrel cage"
  extends Modelica.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines.IMC_Initialize;
  annotation (experiment(
      StopTime=0.60,
      Interval=0.0001,
      Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(0.45, 0.60)}))),
      Documentation(
        info="<html>
<strong>Test example: Steady-State Initialization of an induction machine with squirrel cage</strong><br>
The induction machine with squirrel cage is initialized in steady-state at no-load;
at time tStart a load torque step is applied.<br>
Simulate for 1.5 seconds and plot (versus time):
<ul>
<li><code>currentQuasiRMSSensor|currentQuasiRMSSensorQS.I</code>: (equivalent) RMS stator current</li>
<li><code>imc|imcQS.wMechanical</code>: machine speed</li>
<li><code>imc|imcQS.tauElectrical</code>: machine torque</li>
</ul>
Default machine parameters of model <em>IM_SquirrelCage</em> are used.
</html>"),
    Diagram(graphics={
        Text(
          extent={{20,8},{100,0}},
                  textStyle={TextStyle.Bold},
          textString="%m phase quasi-static"),
                                            Text(
                  extent={{20,-92},{100,-100}},
                  textStyle={TextStyle.Bold},
          textString="%m phase transient")}));
end IMC_Initialize;
