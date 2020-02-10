import pandas as pd
from matplotlib import pyplot as plt
from random import randint

def calculate_cycle_times(cycle):
    """
    Calculates the close time for a
    specific cycle
    cycle is a dataframe
    """
    a = cycle.drop_duplicates(subset=["CMD"], keep="first")
    b = cycle.drop_duplicates(subset=["CMD"], keep="last")
    commanded_open = a.iloc[0]['TIME']
    shutter_opened = b.iloc[0]['TIME']
    commanded_close = a.iloc[-1]['TIME']
    shutter_closed = b.iloc[-1]['TIME']

    open_time = shutter_opened - commanded_open
    close_time = shutter_closed - commanded_close

    return open_time, close_time

def get_all_cycle_times(grouped_data):
    """
    Returns a pandas dataframe like this:
        | cycle | open_time | close_time |
        | 1     | 3.00      | 0.27       |
        |...
    """

    results = pd.DataFrame(columns=['cycle', 'open_time', 'close_time'])

    for cycle_number in range(0, len(grouped_data)):
        cycle = grouped_data.get_group(cycle_number)
        open_time, close_time = calculate_cycle_times(cycle)
        results = results.append({'cycle': cycle_number, 'open_time': open_time, 'close_time': close_time}, ignore_index=True)

    # Set the cycle column as the index
    results.set_index('cycle', inplace=True)
    return results

def examine_random_cycle(grouped_data):
    """
    Selects a cycle at random and prints
    its open and close times
    """
    random_number = randint(0, len(grouped_data))
    cycle = grouped_data.get_group(random_number)

    # Filter out duplicates, which often occur
    # when limit switches jitter
    cycle.drop_duplicates(subset="TIME", keep="last", inplace=True)

    # Rearrange things to make the plot work,
    # then plot it
    cycle_formatted = cycle.set_index("TIME")
    cycle_formatted = cycle_formatted[["CMD", "FAULT"]]

    open_time, close_time = calculate_cycle_times(cycle)

    print("Cycle", random_number)
    print(cycle)
    print("Open time:", open_time)
    print("Close time:", close_time)

def examine_all_cycles(grouped_data):
    """
    Calculates all cycle opening and closing
    times using get_all_cycle_times, then
    plots histograms and prints means and SDs
    """

    # Calculate opening and closing time
    # for every cycle
    results = get_all_cycle_times(grouped_df)
    open_times = results['open_time']
    close_times = results['close_time']

    # Calculate means and standard deviations
    means = results.mean().round(0)
    sds = results.std().round(0)

    # Print means and SDs
    print('Open time:\n\tmean =', means['open_time'], '\n\tsd =', sds['open_time'])
    print('Close time:\n\tmean =', means['close_time'], '\n\tsd =', sds['close_time'])

    # Plot histograms of open and close times
    plt.subplot(1, 2, 1)
    open_times.plot.hist(bins=100)
    plt.title('Opening duration')
    plt.subplot(1, 2, 2)
    close_times.plot.hist(bins=100)
    plt.title('Closing duration')
    plt.show()

df = pd.read_csv("results/repetition_test_2020_02_10_working.txt", index_col=False)

# Split up dataframe - give each cycle its
# own dataframe
grouped_df = df.groupby("CYCLE")

#examine_random_cycle(grouped_df)

examine_all_cycles(grouped_df)
