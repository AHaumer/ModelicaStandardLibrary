within Modelica.Electrical.Polyphase.Lines;
model ShortLine "Model of a short line (single segment)"
  extends Interfaces.TwoPlug;
  import Modelica.Units.SI;
  import Modelica.Constants.small;
  parameter Integer M=sum({k for k in 1:m-1}) "Count of mutual components"
    annotation(Dialog(enable=false));
  parameter SI.Resistance R[m](each final min=small) "Phase resistances";
  parameter SI.Inductance L[m](each final min=small) "Phase inductances";
  parameter SI.Conductance G[m](each final min=small) "Phase conductances to ground";
  parameter SI.Capacitance C[m](each final min=small) "Phase capacitances to ground";
//parameter SI.Inductance Lm[M](each final min=small) "Mutual inductances between phases";
//parameter SI.Conductance Gm[M](each final min=small) "Mutual conductances between phases";
//parameter SI.Capacitance Cm[M](each final min=small) "Mutual capacitances between phases";
  parameter Boolean useReferencePin=false "true: use refPin, false: use internal ground"
    annotation (Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter Boolean useHeatPort=false "= true, if heatPort is enabled"
    annotation (Dialog(tab="Thermal behaviour"), Evaluate=true, HideResult=true, choices(checkBox=true));
  parameter SI.Temperature T_ref=293.15 "Reference temperature"
    annotation (Dialog(tab="Thermal behaviour"));
  parameter SI.Temperature T=293.15
    "Fixed device temperature if useHeatPort = false"
    annotation (Dialog(tab="Thermal behaviour", enable=not useHeatPort));
  parameter SI.LinearTemperatureCoefficient alpha_R=0
    "Temperature coefficient of resistances (R_actual = R*(1 + alpha*(heatPort.T - T_ref))"
    annotation (Dialog(tab="Thermal behaviour"));
  parameter SI.LinearTemperatureCoefficient alpha_G=0
    "Temperature coefficient of conductances (G_actual = G/(1 + alpha*(heatPort.T - T_ref))"
    annotation (Dialog(tab="Thermal behaviour"));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if useHeatPort
    annotation (Placement(transformation(extent={{-110,-110},{-90,-90}}),
        iconTransformation(extent={{-110,-110},{-90,-90}})));
  Modelica.Electrical.Analog.Basic.Ground ground if not useReferencePin "Internal ground"
    annotation (Placement(transformation(extent={{10,-100},{30,-80}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin refPin if useReferencePin "Reference pin"
    annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));
  Basic.Resistor resistor1(
    m=m,
    R=R/2,
    T_ref=fill(T_ref, m),
    alpha=fill(alpha_R, m),
    useHeatPort=useHeatPort,
    T=fill(T, m))
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Basic.Resistor resistor2(
    m=m,
    R=R/2,
    T_ref=fill(T_ref, m),
    alpha=fill(alpha_R, m),
    useHeatPort=useHeatPort,
    T=fill(T, m))
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Basic.Inductor inductor1(m=m, L=L/2)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Basic.Inductor inductor2(m=m, L=L/2)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Basic.Conductor conductor(
    m=m,
    G=G,
    T_ref=fill(T_ref, m),
    alpha=fill(alpha_G, m),
    useHeatPort=useHeatPort,
    T=fill(T_ref, m)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-10,-40})));
  Basic.Capacitor capacitor(m=m, C=C) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={10,-40})));
  Basic.Star star(m=m) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-70})));
equation
  connect(plug_p, resistor1.plug_p)
    annotation (Line(points={{-100,0},{-90,0}}, color={0,0,255}));
  connect(resistor2.plug_n, plug_n)
    annotation (Line(points={{90,0},{100,0}}, color={0,0,255}));
  connect(inductor2.plug_n, resistor2.plug_p)
    annotation (Line(points={{60,0},{70,0}}, color={0,0,255}));
  connect(resistor1.plug_n, inductor1.plug_p)
    annotation (Line(points={{-70,0},{-60,0}}, color={0,0,255}));
  connect(star.pin_n, ground.p)
    annotation (Line(points={{0,-80},{20,-80}}, color={0,0,255}));
  connect(star.pin_n, refPin) annotation (Line(points={{-1.77636e-15,-80},{0,-80},
          {0,-100},{0,-100}}, color={0,0,255}));
  connect(conductor.plug_n, star.plug_p) annotation (Line(points={{-10,-50},{-10,
          -60},{1.77636e-15,-60}}, color={0,0,255}));
  connect(star.plug_p, capacitor.plug_n) annotation (Line(points={{1.77636e-15,-60},
          {10,-60},{10,-50}}, color={0,0,255}));
  connect(inductor1.plug_n, conductor.plug_p)
    annotation (Line(points={{-40,0},{0,0},{0,-20},{-10,-20},{-10,-30}},
                                                         color={0,0,255}));
  connect(inductor2.plug_p, capacitor.plug_p)
    annotation (Line(points={{40,0},{0,0},{0,-20},{10,-20},{10,-30}},
                                                      color={0,0,255}));
  connect(inductor1.plug_n, inductor2.plug_p)
    annotation (Line(points={{-40,0},{40,0}}, color={0,0,255}));
  if useHeatPort then
    for k in 1:m loop
      connect(resistor1.heatPort[m], heatPort);
      connect(resistor2.heatPort[m], heatPort);
      connect(conductor.heatPort[m], heatPort);
    end for;
  end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics={
        Line(points={{-90,0},{90,0}}, color={0,0,255}),
        Rectangle(
          extent={{-86,8},{-54,-8}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-150,90},{150,50}},
          textString="%name",
          textColor={0,0,255}),
        Rectangle(
          extent={{-46,8},{-14,-8}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{54,8},{86,-8}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{14,8},{46,-8}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-16,8},{16,-8}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={-10,-40},
          rotation=90),
        Line(points={{0,0},{-10,-24}}, color={0,0,255}),
        Line(points={{-10,-56},{0,-80}}, color={0,0,255}),
        Line(points={{0,-80},{0,-90}}, color={0,0,255}),
        Line(points={{2,-36},{18,-36}}, color={0,0,255}),
        Line(points={{2,-44},{18,-44}}, color={0,0,255}),
        Line(points={{0,0},{10,-24},{10,-36}}, color={0,0,255}),
        Line(points={{10,-44},{10,-56},{0,-80}}, color={0,0,255})}),
                                  Documentation(info="<html>
<p>
T-Model of a short line.
</p>
</html>"));
end ShortLine;
