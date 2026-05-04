within ModelicaTest.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.SynchronousMachines;
model SMEE_Generator "Electrical excited synchronous machine operating as generator"
  extends Modelica.Magnetic.QuasiStatic.FundamentalWave.Examples.BasicMachines.SynchronousMachines.SMEE_Generator;
  annotation (
    experiment(
      StopTime=15.1,
      Interval=1E-4,
      Tolerance=1e-06),
    TestCase(shouldPass = true,
      __ModelicaAssociation(Comparison(TimeWindows={TimeSlot(14.9, 15.1)}))),
    Documentation(info="<html>
<p>
This example compares a time transient and a quasi-static model of a electrically excited synchronous machine.
The electrically excited synchronous generators are connected to the grid and driven with constant speed.
Since speed is slightly smaller than synchronous speed corresponding to mains frequency,
rotor angle is very slowly increased. This allows to see several characteristics dependent on rotor angle.
</p>

<p>
Simulate for 30 seconds and plot versus <code>rotorAngle|rotorAngleQS.rotorDisplacementAngle</code>:
</p>

<ul>
<li><code>smpm|smpmQS.tauElectrical</code>: machine torque</li>
</ul>

<p>Since the rotor slip is very low the transient and quasi-static electromagnetic torque are practically equal.</p>
</html>"),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                         graphics={         Text(
                  extent={{20,8},{100,0}},
                  fillColor={255,255,170},
                  fillPattern=FillPattern.Solid,
                  textStyle={TextStyle.Bold},
          textString="%m phase quasi-static"),     Text(
          extent={{20,-92},{100,-100}},
                  textStyle={TextStyle.Bold},
                  textString="%m phase transient")}));
end SMEE_Generator;
