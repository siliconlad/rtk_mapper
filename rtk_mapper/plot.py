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
        ax.set_xlabel("")
        for marker in m.markers:
            ax.scatter(marker.pos[0], marker.pos[1], 49, marker.type, edgecolors='k')
        return ax