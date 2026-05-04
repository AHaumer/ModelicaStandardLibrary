within ModelicaTest.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines;
model IMC_Conveyor "Induction machine with squirrel cage and inverter driving a conveyor"
  extends Modelica.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines.IMC_Conveyor;
  annotation (experiment(StopTime=5.00, Interval=0.0001, Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(4.00, 5.00)}))),
    Documentation(
        info="<html>
<p>
An ideal frequency inverter is modeled by using a VfController and a three-phase SignalVoltage.
Frequency is driven by a load cycle of acceleration, constant speed, deceleration and standstill.
The mechanical load is a constant torque like a conveyor (with regularization around zero speed).
</p>
<p>Simulate for 20 seconds and plot (versus time):</p>
<ul>
<li><code>currentQuasiRMSSensor|currentQuasiRMSSensorQS.I</code>: (equivalent) stator current RMS</li>
<li><code>imc|imcQS.wMechanical</code>: machine speed</li>
<li><code>imc|imcQS.tauElectrical</code>: machine torque</li>
</ul>
<p>Default machine parameters are used.</p>
</html>"),
    Diagram(graphics={
        Text(
          extent={{20,60},{100,52}},
                  textStyle={TextStyle.Bold},
          textString="%m phase quasi-static"),
                                            Text(
                  extent={{20,-40},{100,-48}},
                  textStyle={TextStyle.Bold},
          textString="%m phase transient")}));
end IMC_Conveyor;
