import matplotlib.axes

from rtk_mapper.map import RTKMap


class RTKPlotter:
    """Plotting class containing plotting methods to plot RTKMap objects"""

    @staticmethod
    def plot(m: RTKMap, ax: matplotlib.axes.Axes) -> matplotlib.axes.Axes:
        """
        Plots map as a scatter plot.

        :param m: RTK map.
        :param ax: matplotlib axes to plot.
        :returns: matplotlib axes
        """
        ax.axis('equal')
        ax.set_ylabel("")
        ax.set_yticks([])
        ax.set_xlabel("")
        ax.set_xticks([])
        for i, marker in enumerate(m.markers):
            edgecolor = 'red' if (m.selected == i and m.update_mode) else 'k'
            lw = 3.5 if (m.selected == i and m.update_mode) else 2.0
            ax.scatter(marker.pos[0], marker.pos[1], 69, marker.type.hex_code, linewidths=lw, edgecolors=edgecolor)
        return ax
