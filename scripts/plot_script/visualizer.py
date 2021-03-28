# -*- coding: utf-8 -*-
'''
Visualize the comparisons between the various algorithms
on different performance metrics.

The plotting standards as defined in ss_plotting: 
github.com/personalrobotics/ss_plotting

Date: Aug 29th 2018.
'''

# Import standard python libraries

import argparse
# from IPython import embed
from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib.collections import PatchCollection
import numpy
# from ss_plotting import colors, make_plots, plot_utils
import plot_utils

confidence_magnitude = 95

def visualize_planning_times(algorithm_data, densify_data, trivial_data, save_file):
    '''
    Visualize the path lengths comparison plots between DP and SP algorithms

    @param algorithm_data Time running the C-MINT algorithm.
    @param densify_data Time running the DENSIFY algorithm.
    @param trivial_data Time running the TRIVIAL algorithm.
    @param visualize True if just need plots to show. False if need to be saved.
    @param save_file File name for the plots to be saved under.
    '''

    show_plot = True
    if save_file is not None:
        show_plot = False

    print(save_file)

    sample_epsilon = [0.01, 0.005, 0.0025]
    num_trials = numpy.shape(algorithm_data)[1] #971

    # Since there is -1, we are forced to use a for loop

    # initialize
    normalized_algorithm_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    normalized_densify_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    # normalized_trivial_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    valid_algorithm_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]
    valid_densify_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]
    # valid_trivial_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]

    # initialize holders for median
    median_normalized_algorithm_data = []
    median_normalized_densify_data = []
    # median_normalized_trivial_data = []

    # initialize holders for confidence bounds
    upper_interval_algorithm_data = []
    lower_interval_algorithm_data = []

    upper_interval_densify_data = []
    lower_interval_densify_data = []

    # compute the medians and confidence bounds only for valid data
    for i in range(len(sample_epsilon)):

        # Compute the median, upper and lower confidence bounds for algorithm
        valid_current_data = algorithm_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)
        print('LP:',lower_percentile_cutoff)
        print('UP:',upper_percentile_cutoff)
        median_normalized_algorithm_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_algorithm_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_algorithm_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))

        valid_current_data = densify_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)

        median_normalized_densify_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_densify_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_densify_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))


    # embed()


    cutoffa = 0 # diverse
    cutoffs = 0 # bottleneck

    series = [  [sample_epsilon[cutoffa:], median_normalized_algorithm_data[cutoffa:]],
                [sample_epsilon[cutoffs:], median_normalized_densify_data[cutoffs:]]]
    series_colors = ["red", "blue"]

    series_confidence_upper = [upper_interval_algorithm_data[cutoffa:], upper_interval_densify_data[cutoffs:]]
    series_confidence_lower = [lower_interval_algorithm_data[cutoffa:], lower_interval_densify_data[cutoffs:]]
    series_err_colors = ["red", "blue"]

    fig, ax = plot_utils.plot(series, series_colors,
                series_labels = ['new implementation', 'old implementation'],
                series_errs = None,
                series_confidence_lower=series_confidence_lower,
                series_confidence_upper=series_confidence_upper,
                series_err_colors=series_err_colors,
                fill_error=True,
                linewidth=2,
                plot_markers = ['s', 's'],
                marker_sizes = None,
                line_styles = None,
                mark_every=1,
                plot_xlabel = "Epsilon Value", 
                plot_xlim = None,
                x_scale = 'linear',
                plot_ylabel = "Planning Time", 
                plot_ylim = None,
                y_scale = 'log',
                plot_title = 'Comparison of Planning Times',
                fontsize=12, 
                legend_fontsize=12,
                legend_location = 'best',
                show_plot = show_plot,
                savefile = save_file)

def visualize_num_vertices(algorithm_data, densify_data, trivial_data, save_file):
    '''
    Visualize the path lengths comparison plots between DP and SP algorithms

    @param algorithm_data Time running the C-MINT algorithm.
    @param densify_data Time running the DENSIFY algorithm.
    @param trivial_data Time running the TRIVIAL algorithm.
    @param visualize True if just need plots to show. False if need to be saved.
    @param save_file File name for the plots to be saved under.
    '''

    show_plot = True
    if save_file is not None:
        show_plot = False

    print(save_file)

    sample_epsilon = [0.01, 0.005, 0.0025]
    num_trials = numpy.shape(algorithm_data)[1] #971

    # Since there is -1, we are forced to use a for loop

    # initialize
    normalized_algorithm_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    normalized_densify_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    valid_algorithm_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]
    valid_densify_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]

    # initialize holders for median
    median_normalized_algorithm_data = []
    median_normalized_densify_data = []

    # initialize holders for confidence bounds
    upper_interval_algorithm_data = []
    lower_interval_algorithm_data = []

    upper_interval_densify_data = []
    lower_interval_densify_data = []

    # compute the medians and confidence bounds only for valid data
    for i in range(len(sample_epsilon)):

        # Compute the median, upper and lower confidence bounds for algorithm
        valid_current_data = algorithm_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)

        median_normalized_algorithm_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_algorithm_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_algorithm_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))

        valid_current_data = densify_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)

        median_normalized_densify_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_densify_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_densify_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))


    # embed()


    cutoffa = 0 # diverse
    cutoffs = 0 # bottleneck

    series = [  [sample_epsilon[cutoffa:], median_normalized_algorithm_data[cutoffa:]],
                [sample_epsilon[cutoffs:], median_normalized_densify_data[cutoffs:]]]
    series_colors = ["red", "blue"]

    series_confidence_upper = [upper_interval_algorithm_data[cutoffa:], upper_interval_densify_data[cutoffs:]]
    series_confidence_lower = [lower_interval_algorithm_data[cutoffa:], lower_interval_densify_data[cutoffs:]]
    series_err_colors = ["red", "blue"]

    fig, ax = plot_utils.plot(series, series_colors,
                series_labels = ['new implementation', 'old implementation'],
                series_errs = None,
                series_confidence_lower=series_confidence_lower,
                series_confidence_upper=series_confidence_upper,
                series_err_colors=series_err_colors,
                fill_error=True,
                linewidth=2,
                plot_markers = ['s', 's'],
                marker_sizes = None,
                line_styles = None,
                mark_every=1,
                plot_xlabel = "Epsilon Value", 
                plot_xlim = None,
                x_scale = 'linear',
                plot_ylabel = "Number of Vertices", 
                plot_ylim = None,
                y_scale = 'log',
                plot_title = 'Comparison of Number of Vertices in Graph',
                fontsize=12, 
                legend_fontsize=12,
                legend_location = 'best',
                show_plot = show_plot,
                savefile = save_file)

def visualize_num_edges(algorithm_data, densify_data, trivial_data, save_file):
    '''
    Visualize the path lengths comparison plots between DP and SP algorithms

    @param algorithm_data Time running the C-MINT algorithm.
    @param densify_data Time running the DENSIFY algorithm.
    @param trivial_data Time running the TRIVIAL algorithm.
    @param visualize True if just need plots to show. False if need to be saved.
    @param save_file File name for the plots to be saved under.
    '''

    show_plot = True
    if save_file is not None:
        show_plot = False

    print(save_file)

    sample_epsilon = [0.01, 0.005, 0.0025]
    num_trials = numpy.shape(algorithm_data)[1] #971

    # Since there is -1, we are forced to use a for loop

    # initialize
    normalized_algorithm_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    normalized_densify_data = numpy.zeros([len(sample_epsilon), num_trials]) # [3 x 971]
    valid_algorithm_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]
    valid_densify_data = numpy.zeros([len(sample_epsilon), 1]) # [3 x 1]

    # initialize holders for median
    median_normalized_algorithm_data = []
    median_normalized_densify_data = []

    # initialize holders for confidence bounds
    upper_interval_algorithm_data = []
    lower_interval_algorithm_data = []

    upper_interval_densify_data = []
    lower_interval_densify_data = []

    # compute the medians and confidence bounds only for valid data
    for i in range(len(sample_epsilon)):

        # Compute the median, upper and lower confidence bounds for algorithm
        valid_current_data = algorithm_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)

        median_normalized_algorithm_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_algorithm_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_algorithm_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))

        valid_current_data = densify_data[i,:]
        lower_percentile_cutoff, upper_percentile_cutoff = plot_utils.computeConfidenceSplit(num_trials, confidence_magnitude)

        median_normalized_densify_data.append(numpy.percentile(valid_current_data, 50))
        lower_interval_densify_data.append(numpy.percentile(valid_current_data, lower_percentile_cutoff))
        upper_interval_densify_data.append(numpy.percentile(valid_current_data, upper_percentile_cutoff))

    # embed()


    cutoffa = 0 # diverse
    cutoffs = 0 # bottleneck
    cutoffh = 0 # LEGO

    series = [  [sample_epsilon[cutoffa:], median_normalized_algorithm_data[cutoffa:]],
                [sample_epsilon[cutoffs:], median_normalized_densify_data[cutoffs:]]]
    series_colors = ["red", "blue"]

    series_confidence_upper = [upper_interval_algorithm_data[cutoffa:], upper_interval_densify_data[cutoffs:]]
    series_confidence_lower = [lower_interval_algorithm_data[cutoffa:], lower_interval_densify_data[cutoffs:]]
    series_err_colors = ["red", "blue"]

    fig, ax = plot_utils.plot(series, series_colors,
                series_labels = ['new implementation', 'old implementation'],
                series_errs = None,
                series_confidence_lower=series_confidence_lower,
                series_confidence_upper=series_confidence_upper,
                series_err_colors=series_err_colors,
                fill_error=True,
                linewidth=2,
                plot_markers = ['s', 's'],
                marker_sizes = None,
                line_styles = None,
                mark_every=1,
                plot_xlabel = "Epsilon Value", 
                plot_xlim = None,
                x_scale = 'linear',
                plot_ylabel = "Number of Edges in Graph", 
                plot_ylim = None,
                y_scale = 'log',
                plot_title = 'Comparison of Number of Edges in Graph',
                fontsize=12, 
                legend_fontsize=12,
                legend_location = 'best',
                show_plot = show_plot,
                savefile = save_file)

# Main Function
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for comparing planning times')

    parser.add_argument('-v', '--visualize', action='store_false', 
                        help='Set to true to show the plot. Set to false to save the file')

    args = parser.parse_args()

    C_MINT_planning_time_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/planning_times/C_MINT.txt")
    DENSIFY_planning_time_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/planning_times/C_MINT_2.txt")
    TRIVIAL_planning_time_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/planning_times/TRIVIAL.txt")

    C_MINT_num_vertices_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_vertices/C_MINT.txt")
    DENSIFY_num_vertices_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_vertices/C_MINT_2.txt")
    TRIVIAL_num_vertices_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_vertices/TRIVIAL.txt")

    C_MINT_num_edges_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_edges/C_MINT.txt")
    DENSIFY_num_edges_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_edges/C_MINT_2.txt")
    TRIVIAL_num_edges_data = numpy.loadtxt("/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_edges/TRIVIAL.txt")

    if not args.visualize:
        planning_time_save_file = None
        num_vertices_save_file = None
        num_edges_save_file = None

    else:
        planning_time_save_file = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/planning_time.png"
        num_vertices_save_file = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_vertices.png"
        num_edges_save_file = "/home/rajat/melodic_ws/src/C-MINT/data/results/compiled_results/num_edges.png"

    visualize_planning_times(C_MINT_planning_time_data, DENSIFY_planning_time_data, TRIVIAL_planning_time_data, planning_time_save_file)
    visualize_num_vertices(C_MINT_num_vertices_data, DENSIFY_num_vertices_data, TRIVIAL_num_vertices_data, num_vertices_save_file)
    visualize_num_edges(C_MINT_num_edges_data, DENSIFY_num_edges_data, TRIVIAL_num_edges_data, num_edges_save_file)