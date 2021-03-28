#!/usr/bin/env python
from IPython import embed
import numpy
import matplotlib
import matplotlib.pyplot as plt
from scipy.special import comb, erfinv

def computeConfidence(n, lower, upper):
    """
    Computes the distribution-free condifence interval
    @param n The number of samples
    @param lower Lower percentile cut-off
    @param upper Upper percentile cut-off
    Source: http://probabilityandstats.wordpress.com/2010/02/22/confidence-intervals-for-percentiles/
    """

    confidence = 0
    for k in range(lower, upper):
        confidence += comb(n, k) * (0.5**k) * (0.5**(n - k))

    return confidence

def computeConfidenceSplit(n, confidence):
    """
    Computes the percentile split around 50 for given distribution-free condifence interval
    @param n The number of samples
    @param confidence The required confidence in percentage
    """
    confidence /= 100.0
    for offset in range(1,486):
        lower = 485 - offset
        upper = 485 + offset

        current = computeConfidence(n, lower, upper)
        print(n)
        print('C:',lower,upper,current)
        if current > confidence:
            return lower*10/97, upper*10/97

    return 0, 100


def computeBinomialConfidenceInterval(n, success_rate, confidence):
    '''
    Computes the confidence bounds for discrete binomial distribution / trials
    @param n The number of trials
    @param success_rate Success rate as a decimal (#1 / n)
    @param condifence Required confidence in percentage
    Source: https://en.wikipedia.org/wiki/Binomial_proportion_confidence_interval 
    '''
    error = 1 - confidence / 100.0
    p = error/2.0
    z = -numpy.sqrt(2)*erfinv(2*p - 1) # Source: https://en.wikipedia.org/wiki/Probit

    # Symmetric confidence bounds
    return z*numpy.sqrt(success_rate*(1 - success_rate)/n)


def plot(series, series_colors,
         series_labels = None,
         series_errs = None,
         series_confidence_lower = None,
         series_confidence_upper = None,
         series_err_colors = None,
         fill_error=True,
         linewidth = 2,
         plot_markers = None,
         marker_sizes = None,
         line_styles = None,
         mark_every=1,
         plot_xlabel = None, 
         plot_xlim = None,
         x_scale = 'linear',
         plot_ylabel = None, 
         plot_ylim = None,
         y_scale = 'linear',
         plot_title = None,
         fontsize=8, 
         legend_fontsize=8,
         legend_location = 'best',
         show_plot = True,
         savefile = None):
    """
    Plot yvals as a function of xvals
    @param series List of (xvals, yvals) for each series- each of these will be plotted in a different color
    @param series_labels List of labels for each series - same length as series
    @param series_colors List of colors for each series - same length as series
    @param series_color_emphasis List of booleans, one for each series, indicating whether the series
       color should be bold - if None no series is bold
    @param series_errs The error values for each series - if None no error bars are plotted
    @param series_err_colors The colors for the error bars for each series, if None black is used
-   @param fill_error If true, shade the area to show error bars, if not, draw actual error bars
    @param plot_xlabel The label for the x-axis - if None no label is printed
    @param plot_xlim The limits for the x-axis - if None these limits are selected automatically
    @param plot_ylabel The label for the y-axis - if None no label is printed
    @param plot_ylim The limits for the y-axis - if None these limits are selected automatically
    @param plot_title A title for the plot - if None no title is printed
    @param fontsize The size of font for all labels
    @param legend_fontsize The size of font for the legend labels
    @param linewidth The width of the lines in the plot
    @param legend_location The location of the legend, if None no legend is included
    @param savefile The path to save the plot to, if None plot is not saved
    @param savefile_size The size of the saved plot
    @param show_plot If True, display the plot on the screen via a call to plt.show()
    @param x_scale set to log or linear for the x axis
    @param y_scale set to log or linear for the y axis
    @param plot_markers if not None, a list of line marker symbols
    @param line_styles if not None, a list of line style symbols
    @return fig, ax The figure and axis the plot was created in
    """


    matplotlib.rcParams['font.family'] = 'serif'
    matplotlib.rcParams['font.serif'] = 'Palatino'
    matplotlib.rcParams['font.size'] = fontsize
    matplotlib.rcParams['legend.labelspacing'] = 0.5
    matplotlib.rcParams['text.usetex'] = True


    # Validate
    if series is None or len(series) == 0:
        raise ValueError('No data series')
    num_series = len(series)

    if len(series_colors) != num_series:
        raise ValueError('You must define a color for every series')
        
    if series_labels is None:
        series_labels = [None for s in series]
    if len(series_labels) != num_series:
        raise ValueError('You must define a label for every series')

    if series_errs is None:
        series_errs = [None for s in series]
    if len(series_errs) != num_series:
        raise ValueError('series_errs is not None. Must provide error value for every series.')

    if series_confidence_lower is None:
        series_confidence_lower = [None for s in series]
    if len(series_confidence_lower) != num_series:
        raise ValueError('series_confidence_lower is not None. Must provide error value for every series.')

    if series_confidence_upper is None:
        series_confidence_upper = [None for s in series]
    if len(series_confidence_upper) != num_series:
        raise ValueError('series_confidence_upper is not None. Must provide error value for every series.')

    if series_err_colors is None:
        series_err_colors = ['black' for s in series]
    if len(series_err_colors) != num_series:
        raise ValueError('Must provide an error bar color for every series')

    if plot_markers is None:
        plot_markers = ['None' for s in series]
    if len(plot_markers ) != num_series:
        raise ValueError('The marker list must contain all series')
        
    if marker_sizes is None:
        marker_sizes = [6 for s in series]
    if len(marker_sizes) != num_series:
        raise ValueError('The markersize list must contain a size for every series')

    if line_styles is None:
        line_styles  = ['-' for s in series]
    if len(line_styles) != num_series:
        raise ValueError('The line style list must contain all series')

    fig, ax = plt.subplots()
    # configure_fonts(fontsize=fontsize, legend_fontsize=legend_fontsize)

    num_series = len(series)
    for idx in range(num_series):
        xvals = series[idx][0]
        yvals = numpy.array(series[idx][1])

        r = ax.plot(xvals, yvals,
                    label = series_labels[idx],
                    color = series_colors[idx], 
                    marker = plot_markers[idx],
                    markersize = marker_sizes[idx],
                    markerfacecolor = 'white',
                    markeredgecolor = series_colors[idx],
                    linestyle = line_styles[idx],
                    lw = linewidth,
                    markevery=mark_every)

        errs = series_errs[idx]
        confidence_upper = series_confidence_upper[idx]
        confidence_lower = series_confidence_lower[idx]
        if errs is not None:
            shade_color = series_err_colors[idx]
            if fill_error:
                plot_utils.shaded_error(ax, xvals, yvals, numpy.array(errs), color=shade_color)
            else:
                ax.errorbar(xvals, yvals, yerr=errs, linestyle='None', ecolor=shade_color)
        if confidence_lower is not None and confidence_upper is not None:
            confidence_levels = [confidence_lower, confidence_upper]
            shade_color = series_err_colors[idx]
            if fill_error:
                ax.fill_between(xvals, numpy.array(confidence_lower), numpy.array(confidence_upper), color=shade_color, alpha = 0.1, linewidth=2, edgecolor=series_colors[idx])
            else:
                ax.errorbar(xvals, yvals, yerr=confidence_levels, linestyle='None', ecolor=shade_color)
        
    
    # Label the plot
    if plot_ylabel is not None:
        ax.set_ylabel(plot_ylabel)
    if plot_xlabel is not None:
        ax.set_xlabel(plot_xlabel)
    if plot_title is not None:
        ax.set_title(plot_title)
    if plot_xlim is not None:
        ax.set_xlim(plot_xlim)
    if plot_ylim is not None:
        ax.set_ylim(plot_ylim)
    
    ax.set_yscale(y_scale)
    ax.set_xscale(x_scale)


    # Legend
    if legend_location is not None:
        ax.legend(loc=legend_location, frameon=False)

    # Show
    if show_plot:
        plt.show()

    if savefile is not None:
        fig.savefig(savefile, pad_inches=0.02, bbox_inches='tight')

    return fig, ax

if __name__ == '__main__':
    a = computeConfidence(100, 40, 60)
    embed()

    l, u = computeConfidenceSplit(100, 95)
    embed()

    a = computeBinomialConfidenceInterval(1,1,95)