import plotly.plotly as py
import plotly.offline
import plotly.figure_factory as ff

df = [dict(Task="Job A", Start=10, Finish=30, Complete=100),
      dict(Task="Job B", Start=10, Finish=30, Complete=100),
      dict(Task="Job C", Start=10, Finish=30, Complete=100)]

fig = ff.create_gantt(df, colors='Viridis', index_col='Complete', show_colorbar=False)
plotly.offline.plot(fig)