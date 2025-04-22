#!/bin/bash

# Directories to include in formatting
INCLUDE_DIRS=(
"src"
"include"
)

# Directories to exclude from formatting
EXCLUDE_DIRS=(
"external"
"build"
"libs"
"test"
"install"
"log"
"resources"
"autoware"
)

# File extensions to format
EXTENSIONS=("cpp" "hpp" "h" "cc")

# Function to print usage
print_usage() {
    echo "Usage: $0 <path>"
    echo "  <path> can be a directory or a specific file"
}

# Check if an argument is provided
if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

INPUT_PATH="$1"

# Check if the input path exists
if [ ! -e "$INPUT_PATH" ]; then
    echo "Error: The specified path does not exist."
    exit 1
fi

# If the input is a file, format it directly
if [ -f "$INPUT_PATH" ]; then
    extension="${INPUT_PATH##*.}"
    if [[ " ${EXTENSIONS[@]} " =~ " ${extension} " ]]; then
        clang-format -i -style=file "$INPUT_PATH"
        echo "Formatted file: $INPUT_PATH"
    else
        echo "Skipped file with unsupported extension: $INPUT_PATH"
    fi
    exit 0
fi

# If the input is a directory, proceed with recursive search
if [ ! -d "$INPUT_PATH" ]; then
    echo "Error: The specified path is neither a file nor a directory."
    exit 1
fi

# Build the find command
FIND_CMD="find \"$INPUT_PATH\""

# Add directories to include
FIND_CMD+=" \( -false"
for dir in "${INCLUDE_DIRS[@]}"; do
    FIND_CMD+=" -o -path '*/$dir/*'"
done
FIND_CMD+=" \)"

# Add file extensions
FIND_CMD+=" \( -false"
for ext in "${EXTENSIONS[@]}"; do
    FIND_CMD+=" -o -name '*.$ext'"
done
FIND_CMD+=" \)"

# Add directories to exclude
for dir in "${EXCLUDE_DIRS[@]}"; do
    FIND_CMD+=" -not -path '*/$dir/*'"
done

# Run clang-format
eval $FIND_CMD | xargs clang-format -i -style=file
echo "Formatting complete!"
