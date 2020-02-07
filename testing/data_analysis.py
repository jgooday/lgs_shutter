import pandas as pd

df = pd.read_csv("results/prelim_test_2020_02_07.txt", nrows=1870, index_col=False)

print(df)
