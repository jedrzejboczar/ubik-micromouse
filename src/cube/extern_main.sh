#!/bin/bash

HERE=$(dirname "$(realpath ${BASH_SOURCE[0]})")

main_path="$(find $HERE -name 'main\.c')"

if ! [[ -f "$main_path" ]]; then
    echo "Error: Could not find main.c"
    exit 1
fi

# replace part of file between lines matched by start and end pattern
# with the given replacement string
# Usage: replace_in_file FILENAME START_PATTERN END_PATTERN REPLACEMENT
replace_in_file() {
    local file="$1"
    local star_pattern="$2"
    local end_pattern="$3"
    local replacement="$4"

    # replace newlines with literal newline (for sed compatibility)
    replacement=$(echo "$replacement" | awk '{printf "%s\\n", $0}')

    # replace part of file between lines matched by the pattern
    # with the given replacement
    sed -ie "/$star_pattern/,/$end_pattern/c\ $replacement" "$file"
}


# replaces USER CODE section 2 with this:
replacement=$(cat << EOF
 /* USER CODE BEGIN 2 */

  /* Pass control to extern_main() defined somewhere else with C linkage */
  extern void extern_main(void);
  extern_main();

  /* Execution should never get here */
  Error_Handler();

  /* USER CODE END 2 */
EOF
)

replace_in_file "$main_path" 'USER CODE BEGIN 2' 'USER CODE END 2' "$replacement"

