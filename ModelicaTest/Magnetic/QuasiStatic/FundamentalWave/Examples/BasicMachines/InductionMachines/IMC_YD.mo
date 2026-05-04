within ModelicaTest.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines;
model IMC_YD "Induction machine with squirrel cage starting Y-D"
  extends Modelica.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.InductionMachines.IMC_YD;
  annotation (experiment(StopTime=2.1,Interval=0.0001,Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(2.00, 2.10)}))),
    Documentation(
        info="<html>
<p>
The original documentation is available at the model from which this one is extended.
</p>
</html>"),
    Diagram(graphics={
        Text(
          extent={{-60,20},{20,12}},
                  textStyle={TextStyle.Bold},
          textString="%m phase quasi-static"), Text(
                  extent={{-60,-80},{20,-88}},
                  textStyle={TextStyle.Bold},
                  textString="%m phase transient")}));
end IMC_YD;
