within ModelicaTest.Electrical.PowerConverters.Examples.ACDC.RectifierCenterTap2Pulse;
model ThyristorCenterTap2Pulse_RLV_Characteristic
  "Characteristic of two pulse thyristor rectifier with center tap and R-L load and voltage"
  extends Modelica.Electrical.PowerConverters.Examples.ACDC.RectifierCenterTap2Pulse.ThyristorCenterTap2Pulse_RLV_Characteristic;
  annotation (
    experiment(
      StopTime=5.05,
      Tolerance=1e-06,
      Interval=0.0002),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(4.95, 5.05)}))),
    Documentation(info="<html>
<p>
The original documentation is available at the model from which this one is extended.
</p>
</html>"));
end ThyristorCenterTap2Pulse_RLV_Characteristic;
