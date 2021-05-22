within Modelica.Electrical.Polyphase.Lines;
model Line "Model of a long line"
  extends Interfaces.TwoPlug;
  import Modelica.Units.SI;
  import Modelica.Constants.small;
  parameter Integer M=sum({k for k in 1:m-1}) "Count of mutual components"
    annotation(Dialog(enable=false));
  parameter Integer N(final min=2) "Count of segments";
  parameter SI.Length length(final min=small) "Total lenght of line";
  parameter Real r[m](each final min=small, each unit="Ohm/m")=fill(0.5e-3, m) "Phase resistances/m";
  parameter Real l[m](each final min=small, each unit="H/m")=fill(1e-6, m) "Phase inductances/m";
  parameter Real g[m](each final min=small, each unit="S/m")=fill(0.1e-9, m) "Phase conductances/m to ground";
  parameter Real c[m](each final min=small, each unit="F/m")=fill(10e-12, m) "Phase capacitances/m to ground";
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
  Modelica.Electrical.Analog.Interfaces.NegativePin refPin if useReferencePin "Reference pin"
    annotation (Placement(transformation(extent={{10,-110},{-10,-90}})));
  ShortLine shortLine[N](
    each m=m,
    each R=r*length/N,
    each L=l*length/N,
    each G=g*length/N,
    each C=c*length/N,
    each useReferencePin=true,
    each useHeatPort=true,
    each T_ref=T_ref,
    each T=T,
    each alpha_R=alpha_R,
    each alpha_G=alpha_G)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  connect(plug_p, shortLine[1].plug_p)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,0,255}));
  connect(shortLine[N].plug_n, plug_n)
    annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
  for k in 1:N-1 loop
    connect(shortLine[k].plug_n, shortLine[k+1].plug_p);
  end for;
  if useReferencePin then
    for k in 1:N loop
      connect(shortLine[k].refPin, refPin);
    end for;
  end if;
  if useHeatPort then
    for k in 1:N loop
      connect(shortLine[k].heatPort, heatPort);
    end for;
  end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-150,90},{150,50}},
          textString="%name",
          textColor={0,0,255}),
        Line(points={{-90,0},{90,0}}, color={0,0,255}),
        Rectangle(
          extent={{-86,8},{-54,-8}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
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
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Line;
