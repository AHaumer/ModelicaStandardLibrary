within ModelicaTest.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines;
model IMC_Transformer "Induction machine with squirrel cage starting with transformer"
  extends Modelica.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines.IMC_Transformer;

  annotation (experiment(StopTime=2.1, Interval=0.0001, Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(2.00, 2.10)}))),
    Documentation(
        info="<html>
<p>At start time tStart1 three-phase voltage is supplied to the induction machine with squirrel cage via the transformer;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed;
at start time tStart2 the machine is fed directly from the voltage source, finally reaching nominal speed.</p>
<p>
Simulate for 2.5 seconds and plot (versus time):</p>

<ul>
<li><code>currentQuasiRMSSensor|currentQuasiRMSSensorQS.I</code>: (equivalent) stator current RMS</li>
<li><code>imc|imcQS.wMechanical</code>: machine speed</li>
<li><code>imc|imcQS.tauElectrical</code>: machine torque</li>
</ul>
<p>Default machine parameters are used.</p>
</html>"),
    Diagram(graphics={
        Text(
          extent={{80,8},{160,0}},
                  textStyle={TextStyle.Bold},
          textString="%m phase quasi-static"),
                                            Text(
                  extent={{80,-92},{160,-100}},
                  textStyle={TextStyle.Bold},
          textString="%m phase transient")}));
end IMC_Transformer;
