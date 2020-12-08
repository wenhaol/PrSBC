function h = plot_circle(x,y,r, varargin)

parser = inputParser;
parser.addParameter('Color', 'k');
parser.addParameter('LineStyle', '-');
parser.addParameter('LineWidth', 2);
parse(parser, varargin{:})

color = parser.Results.Color;
linestyle = parser.Results.LineStyle;
linewidth = parser.Results.LineWidth;

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,'Color',color,'LineStyle',linestyle,'LineWidth',linewidth);
hold off
end