within ModelicaTest.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines.ComparisonPolyphase;
model SMEE_Generator_Polyphase
  "Electrical excited polyphase synchronous machine operating as generator"
  extends Modelica.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines.ComparisonPolyphase.SMEE_Generator_Polyphase;
  annotation (experiment(StopTime=30, Interval=0.001, Tolerance=1e-07),
    TestCase(shouldPass = true,
    __ModelicaAssociation(Comparison(timeWindows={TimeWindow(beg, end)}))));
end SMEE_Generator_Polyphase;
