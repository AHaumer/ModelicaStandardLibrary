within ModelicaTest.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines;
model SMEE_LoadDump
  "Test example: ElectricalExcitedSynchronousMachine with voltage controller"
  extends Modelica.Magnetic.FundamentalWave.Examples.BasicMachines.SynchronousMachines.SMEE_LoadDump;
  annotation (experiment(StopTime=10, Interval=1E-4, Tolerance=1e-06),
    TestCase(shouldPass = true,
    __ModelicaAssociation(Comparison(timeWindows={TimeWindow(beg, end)}))));
end SMEE_LoadDump;
