#!/bin/bash

# Check if the file path is provided
if [ $# -ne 1 ]; then
  echo "Usage: $0 <json_file>"
  exit 1
fi

json_file=$1

# Check if the file exists
if [ ! -f "$json_file" ]; then
  echo "Error: File '$json_file' not found."
  exit 1
fi

# Use jq to toggle between "Odom" and "Random"
tmp_file=$(mktemp)

jq '.init_type |= if . == "Odom" then "Random" else "Odom" end' "$json_file" > "$tmp_file"

# Check if jq ran successfully
if [ $? -eq 0 ]; then
  mv "$tmp_file" "$json_file"
  echo "Successfully toggled the init_type value."
else
  echo "Error: Failed to update JSON."
  rm "$tmp_file"
  exit 1
fi
