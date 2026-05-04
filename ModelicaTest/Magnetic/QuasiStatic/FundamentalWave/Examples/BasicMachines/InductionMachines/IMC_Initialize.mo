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
<p>
The original documentation is available at the model from which this one is extended.
</p>
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
