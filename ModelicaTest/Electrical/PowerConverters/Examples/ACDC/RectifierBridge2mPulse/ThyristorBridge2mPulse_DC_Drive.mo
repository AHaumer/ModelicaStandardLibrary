within ModelicaTest.Electrical.PowerConverters.Examples.ACDC.RectifierBridge2mPulse;
model ThyristorBridge2mPulse_DC_Drive
  "2*m pulse thyristor bridge feeding a DC drive"
  extends Modelica.Electrical.PowerConverters.Examples.ACDC.RectifierBridge2mPulse.ThyristorBridge2mPulse_DC_Drive;
  annotation (
    experiment(
      StopTime=2.55,
      Interval=0.0002,
      Tolerance=1e-006),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(2.45, 2.55)}))),
    Documentation(info="<html>
<p>
The original documentation is available at the model from which this one is extended.
</p>
</html>"));
end ThyristorBridge2mPulse_DC_Drive;
