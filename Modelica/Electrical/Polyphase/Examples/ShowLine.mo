within Modelica.Electrical.Polyphase.Examples;
model ShowLine "Demonstrate usage of Line model"
  extends Icons.Example;
  import Modelica.Units.SI;
  parameter Integer m(final min=2)=3 "Number of phases";
  parameter SI.Resistance R=0.1 "Resistance / 1 km";
  parameter SI.Inductance L=1e-3 "Inductance / 1 km";
  parameter SI.Conductance G=1e-6 "Conductance / 1 km";
  parameter SI.Capacitance C=200e-9 "Capacitance / 1 km";
  Sources.SineVoltage sineVoltage(
    m=m,
    V=fill(sqrt(2/3)*400, m),
    f=fill(50, m))                annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-10})));
  Basic.Star starS(m=m) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-40})));
  Basic.Resistor resistor(
    m=m,
    R=fill(10, m),
    T_ref=fill(293.15, m),
    alpha=zeros(m),
    T=fill(293.15, m)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-10})));
  Basic.Star starL(m=m) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={60,-40})));
  Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
  Lines.ShortLine shortLine1(
    m=m,
    R=fill(R, m),
    L=fill(L, m),
    G=fill(G, m),
    C=fill(C, m),
    useReferencePin=false)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Lines.ShortLine shortLine2(
    m=m,
    R=fill(R, m),
    L=fill(L, m),
    G=fill(G, m),
    C=fill(C, m),
    useReferencePin=false)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Lines.ShortLine shortLine3(
    m=m,
    R=fill(R, m),
    L=fill(L, m),
    G=fill(G, m),
    C=fill(C, m),
    useReferencePin=false)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
equation
  connect(sineVoltage.plug_n, starS.plug_p)
    annotation (Line(points={{-60,-20},{-60,-30}}, color={0,0,255}));
  connect(resistor.plug_n, starL.plug_p)
    annotation (Line(points={{60,-20},{60,-30}}, color={0,0,255}));
  connect(starS.pin_n, ground.p)
    annotation (Line(points={{-60,-50},{0,-50}}, color={0,0,255}));
  connect(sineVoltage.plug_p, shortLine1.plug_p)
    annotation (Line(points={{-60,0},{-40,0}}, color={0,0,255}));
  connect(shortLine1.plug_n, shortLine2.plug_p)
    annotation (Line(points={{-20,0},{-10,0}}, color={0,0,255}));
  connect(shortLine2.plug_n, shortLine3.plug_p)
    annotation (Line(points={{10,0},{20,0}}, color={0,0,255}));
  connect(shortLine3.plug_n, resistor.plug_p)
    annotation (Line(points={{40,0},{60,0}}, color={0,0,255}));
  annotation (experiment(
      Interval=0.0001,
      Tolerance=1e-06));
end ShowLine;
