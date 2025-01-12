"""Python script to read a file and extract the experimental results
"""

import pandas as pd
import numpy as np
import os
from typing import List

name_mapping = {
    "mrclam2.pyfg": "MR.CLAM 2",
    "mrclam4.pyfg": "MR.CLAM 4",
    "mrclam6.pyfg": "MR.CLAM 6",
    "mrclam7.pyfg": "MR.CLAM 7",
    "plaza1.pyfg": "Plaza 1",
    "plaza2.pyfg": "Plaza 2",
    "single_drone.pyfg": "Single Drone",
    "tiers.pyfg": "TIERS",
}


def get_lines_with_start_phrase(file_path: str, start_phrase: str):
    """Reads a file and returns the lines that start with a given phrase

    Args:
        file_path (str): Path to the file to read
        start_phrase (str): Phrase that the lines should start with

    Returns:
        list: List of lines that start with the given phrase
    """
    with open(file_path, "r") as file:
        lines = file.readlines()
    return [line for line in lines if line.startswith(start_phrase)]


def make_table(lines: List[str]) -> pd.DataFrame:
    """Converts a list of lines into a pandas DataFrame

    specifically, lines are of the CSV format:
    Experiment result, name: data/plaza2.pyfg, time: 3.57132 seconds, cost: 734.328, marginalized: 0, init rank jump: 0, init random: 0

    Args:
        lines (list): List of lines to convert

    Returns:
        pd.DataFrame: DataFrame with the data
    """
    headers = ["name", "time", "cost", "marginalized", "init rank jump", "init random"]

    # for all lines, drop the first phrase and split the rest by comma
    data = [line.split(",")[1:] for line in lines]
    data = [list(map(lambda x: x.split(": ")[1], line)) for line in data]
    return pd.DataFrame(data, columns=headers)


def clean_df(df: pd.DataFrame) -> pd.DataFrame:
    """Cleans the DataFrame by converting the columns to the correct types

    Args:
        df (pd.DataFrame): DataFrame to clean

    Returns:
        pd.DataFrame: Cleaned DataFrame
    """

    # change names to only have the file name
    df["name"] = df["name"].apply(lambda x: os.path.basename(x))

    # in time column, remove the ' seconds' string
    df["time"] = df["time"].apply(lambda x: x.replace(" seconds", ""))

    df["time"] = df["time"].apply(float)
    df["cost"] = df["cost"].apply(float)
    df["init rank jump"] = df["init rank jump"].apply(int)

    # marginalized and init random should be "True" or "False"
    df["marginalized"] = df["marginalized"].apply(lambda x: int(x) == 1)
    df["init random"] = df["init random"].apply(lambda x: int(x) == 1)

    # order columns: name, marginalized, init random, init rank jump, time, cost
    df = df[["name", "marginalized", "init random", "init rank jump", "time", "cost"]]

    # sort with first priority on name, second on marginalized, third on init random
    df = df.sort_values(["name", "marginalized", "init random"])

    # replace names ['mrclam2.pyfg' 'mrclam4.pyfg' 'mrclam6.pyfg' 'mrclam7.pyfg' 'plaza1.pyfg' 'plaza2.pyfg' 'single_drone.pyfg' 'tiers.pyfg']

    # order names to be: Single Drone, Plaza 2, Plaza 1, TIERS, MR.CLAM 6, MR.CLAM 7, MR.CLAM 4, MR.CLAM 2
    df["name"] = df["name"].apply(lambda x: name_mapping[x])
    df["name"] = pd.Categorical(
        df["name"],
        categories=[
            "Single Drone",
            "Plaza 2",
            "Plaza 1",
            "TIERS",
            "MR.CLAM 6",
            "MR.CLAM 7",
            "MR.CLAM 4",
            "MR.CLAM 2",
        ],
    )

    # drop anything with init rank jump > 2
    df = df[df["init rank jump"] <= 2]

    return df


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Parse data from a file")
    parser.add_argument("file", type=str, help="File to parse")

    args = parser.parse_args()
    file_path = args.file

    # get the lines that start with the phrase
    lines = get_lines_with_start_phrase(file_path, "Experiment result")
    table = make_table(lines)
    table = clean_df(table)
    print(table)

    # for each name, print the rows
    print()
    for name in table["name"].unique():
        subtable = table[table["name"] == name]

        subtable = subtable.sort_values(["marginalized", "time"])
        # print(subtable)

        # for the two entries with the same "marginalized" state, take the one with the lower time
        subtable = subtable.drop_duplicates(["marginalized"])

        # drop "init random" and "cost"
        subtable = subtable.drop(columns=["init random", "cost", "init rank jump"])

        # compute the ratio of the marginalized to the non marginalized time (second entry / first entry)
        subtable = subtable.reset_index(drop=True)
        # subtable["ratio"] = subtable["time"].shift(-1) / subtable["time"]
        subtable["ratio (%)"] =  subtable["time"] / subtable["time"].shift(-1)
        # make ratio a percentage (multiply by 100) and round to 1 decimal place
        subtable["ratio (%)"] = subtable["ratio (%)"].apply(lambda x: round(x * 100, 1))

        # round time to 2 decimal places
        subtable["time"] = subtable["time"].apply(lambda x: round(x, 2))

        print(subtable)
        print()
