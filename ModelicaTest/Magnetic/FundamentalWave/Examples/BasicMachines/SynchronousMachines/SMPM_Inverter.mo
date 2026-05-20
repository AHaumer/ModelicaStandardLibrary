within ModelicaTest.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines;
model SMPM_Inverter
  "Starting of permanent magnet synchronous machine with inverter"
  extends Modelica.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines.SMPM_Inverter;
  annotation (experiment(StopTime=1.5, Interval=1E-4, Tolerance=1e-006),
    TestCase(shouldPass = true,
    __ModelicaAssociation(Comparison(timeWindows={TimeWindow(beg, end)}))));
end SMPM_Inverter;
