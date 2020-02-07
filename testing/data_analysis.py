import pandas as pd
from matplotlib import pyplot

def calculate_close_time(cycle):
    """
    Calculates the close time for a
    specific cycle
    cycle is a dataframe
    """
    a = cycle.drop_duplicates(subset=["CMD"], keep="first")
    b = cycle.drop_duplicates(subset=["CMD"], keep="last")
    print(a)
    print(b)

df = pd.read_csv("results/prelim_test_2020_02_07.txt", nrows=1870, index_col=False)

# Split up dataframe - give each cycle its
# own dataframe
grouped_df = df.groupby("CYCLE")

# Pick a particular cycle to look at
sample = grouped_df.get_group(100)

# Filter out duplicates, which often occur
# when limit switches jitter
sample.drop_duplicates(subset="TIME", keep="last", inplace=True)

# Rearrange things to make the plot work,
# then plot it
sample_formatted = sample.set_index("TIME")
sample_formatted = sample_formatted[["CMD", "FAULT"]]
#sample_formatted.plot()
#pyplot.show()

calculate_close_time(sample)
