within ModelicaTest.Electrical.PowerConverters.Examples.ACAC;
model Dimmer_R "Dimmer with resistive load"
  extends Modelica.Electrical.PowerConverters.Examples.ACAC.Dimmer_R;
  annotation (experiment(
      StopTime=3.00,
      Interval=0.0002,
      Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(2.90, 3.00)}))),
    Documentation(info="<html>
<p>
The original documentation is available at the model from which this one is extended.
</p>
</html>"));
end Dimmer_R;
