class PlotboxArgs:
    def __init__(self, title=None, plots=None, legend=True, time_window=15.0, max_length=None,
                 axis_color='w', axis_width=1, labels=None, plot_hues=4,
                 plot_min_hue=0, plot_max_hue=270, plot_min_value=200, plot_max_value=255,
                 is_angle=False, rad2deg=False):
        # Define title
        if title is not None:
            self.title = title
        elif plots is not None:
            self.title = plots[0]
        else:
            raise ValueError('Must provide a plotbox title or plot names.')

        # Read in plots
        self.plots = []
        if plots is not None:
            for p in plots:
                if isinstance(p, PlotArgs):
                    if p.max_length is None:
                        p.max_length = max_length
                    p.is_angle = is_angle
                    p.rad2deg = rad2deg
                    self.plots.append(p)
                elif isinstance(p, str):
                    self.plots.append(PlotArgs(p, max_length=max_length, is_angle=is_angle, rad2deg=rad2deg))
                else:
                    raise TypeError('plots input {} of incorrect type ({}). Expected PlotArgs or str object'.format(p, type(p)))
        else:
            # If no plots are defined, assume the title and plot are the same
            self.plots.append(PlotArgs(self.title, max_length=max_length, is_angle=is_angle, rad2deg=rad2deg))

        # Save other params
        self.legend         = legend
        self.time_window    = time_window
        self.axis_color     = axis_color
        self.axis_width     = axis_width
        self.labels         = labels
        self.plot_hues      = max(plot_hues, len(self.plots))
        self.plot_min_hue   = plot_min_hue
        self.plot_max_hue   = plot_max_hue
        self.plot_min_value = plot_min_value
        self.plot_max_value = plot_max_value

class PlotArgs:
    def __init__(self, name=None, states=None, sigma_states=None, sigma_bound=1,
                    is_angle=False, rad2deg=False, max_length=None,
                    color=None, hidden=False):
        # Define name
        if name is not None:
            self.name = name
        elif states is not None:
            self.name = states[0]
        else:
            raise ValueError('Must provide a plot name or state names.')

        # Define states
        if states is not None:
            self.state_names = states
        else:
            self.state_names = [self.name]

        # Read in other args
        self.sigma_states = sigma_states
        self.sigma_bound = sigma_bound
        self.is_angle = is_angle or rad2deg
        self.rad2deg = rad2deg
        self.max_length = max_length
        self.color = color
        self.hidden = hidden

    def set_color(self, color):
        self.color = color
