within ModelicaTest.Magnetic.FundamentalWave.Examples.BasicMachines.InductionMachines;
model IMS_Start
  "Starting of induction machine with slip rings"
  extends Modelica.Magnetic.FundamentalWave.Examples.BasicMachines.InductionMachines.IMS_Start;
  annotation (experiment(StopTime=1.5, Interval=1E-4, Tolerance=1e-06),
    TestCase(shouldPass = true,
    __ModelicaAssociation(Comparison(timeWindows={TimeWindow(beg, end)}))));
end IMS_Start;
