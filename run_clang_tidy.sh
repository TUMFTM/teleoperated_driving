#!/bin/bash

debug_info() {
    echo "DEBUG: $1"
}

print_usage() {
    echo "Usage: $0 <path>"
    echo "  <path> can be a directory or a specific file"
}

if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

INPUT_PATH="$1"

if [ ! -e "$INPUT_PATH" ]; then
    echo "Error: The specified path does not exist."
    exit 1
fi

PROJECT_ROOT=$(realpath "$(pwd)")
debug_info "Project root: $PROJECT_ROOT"

build_folder="$PROJECT_ROOT/build"
if [ ! -d "$build_folder" ]; then
    echo "Error: build folder not found at $build_folder"
    exit 1
fi

COMPILE_COMMANDS="$build_folder/compile_commands.json"
if [ ! -f "$COMPILE_COMMANDS" ]; then
    echo "Error: compile_commands.json not found in $build_folder"
    exit 1
fi
debug_info "Using compilation database: $COMPILE_COMMANDS"

EXCLUDE_DIRS=(
    "rclcpp"
    "external"
    "build"
    "install"
    "libs"
    "test"
    "log"
    "resources"
    "autoware"
    "deps"
    "glad"
    "extern"
    "lib"
    "dearImgui"
)

EXTENSIONS=("cpp" "hpp" "h" "cc")

if [ -f "$INPUT_PATH" ]; then
    extension="${INPUT_PATH##*.}"
    if [[ " ${EXTENSIONS[@]} " =~ " ${extension} " ]]; then
        clang-tidy-18 "$INPUT_PATH" \
            -p="$COMPILE_COMMANDS" \
            --extra-arg=-std=c++17 \
            --quiet \
            --header-filter="^$PROJECT_ROOT/(src|include).*"
        echo "Tidied file: $INPUT_PATH"
    else
        echo "Skipped file with unsupported extension: $INPUT_PATH"
    fi
    exit 0
fi

if [ ! -d "$INPUT_PATH" ]; then
    echo "Error: The specified path is neither a file nor a directory."
    exit 1
fi

FIND_CMD="find \"$INPUT_PATH\""

for dir in "${EXCLUDE_DIRS[@]}"; do
    FIND_CMD+=" -type d -name $dir -prune -o"
done

FIND_CMD+=" -type f \("
for ext in "${EXTENSIONS[@]}"; do
    FIND_CMD+=" -name \"*.$ext\" -o"
done
FIND_CMD="${FIND_CMD% -o} \) -print0"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="clang_tidy_output_${TIMESTAMP}.txt"

# Define the awk script
AWK_SCRIPT='
BEGIN { current_file=""; in_error=0; buffered_lines=""; }
/^[^ ]/ {
    if (current_file != "" && in_error && buffered_lines != "") {
        print buffered_lines;
    }
    current_file=$0;
    in_error=0;
    buffered_lines="";
    if ($0 !~ /^\/opt\/ros/ && $0 !~ /^\/usr\/include/) {
        print;
        in_error=1;
    }
}
/^ / {
    if (in_error) {
        buffered_lines = buffered_lines $0 "\n";
    }
}
END {
    if (current_file != "" && in_error && buffered_lines != "") {
        print buffered_lines;
    }
}'

eval $FIND_CMD | xargs -0 -I{} bash -c "
clang-tidy-18 \"{}\" \
-p=\"$COMPILE_COMMANDS\" \
--extra-arg=-std=c++17 \
--quiet \
--header-filter=\"^$PROJECT_ROOT/(src|include).*\" \
2>&1 | awk '$AWK_SCRIPT'
" | tee "$OUTPUT_FILE"

echo "Tidying complete! Output saved to $OUTPUT_FILE"
