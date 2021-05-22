within Modelica.Electrical.Polyphase;
package Lines "Models of polyphase lines"
extends Icons.Package;

annotation (Icon(graphics={
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
        Line(points={{2,-36},{18,-36}}, color={0,0,255}),
        Line(points={{2,-44},{18,-44}}, color={0,0,255}),
        Line(points={{0,0},{10,-24},{10,-36}}, color={0,0,255}),
        Line(points={{10,-44},{10,-56},{0,-80}}, color={0,0,255})}));
end Lines;
