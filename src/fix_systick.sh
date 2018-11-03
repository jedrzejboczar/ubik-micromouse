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
    # awk: for each line print the line (without newline),
    #      for each line after the first, print (before this one) a literal newline
    #      this is to avoid additional newline at the end (to leave rest of file intact)
    replacement=$(echo "$replacement" | awk '{ if (NR > 1) { printf "\\n" }; printf "%s", $0 }')

    # replace part of file between lines matched by the pattern
    # with the given replacement
    sed -ie "/$star_pattern/,/$end_pattern/c\ $replacement" "$file"
}

# replace USER CODE Init with this (to disable systick)
replacement=$(cat << EOF
  /* USER CODE BEGIN SysInit */

  /* Disable SysTick before it shoots!
   *
   * In FreeRTOSConfig.h SysTick_Handler is mapped to xPortSysTickHandler
   * so we cannot let SysTick fire its interrupt before starting FreeRTOS
   * scheduler. In Cube we changed System Timebase to some other timer, so
   * we can safely disable SysTick here. FreeRTOS will configure it when
   * it starts.  */
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);

  /* USER CODE END SysInit */
EOF
)

replace_in_file "$main_path" 'USER CODE BEGIN SysInit' 'USER CODE END SysInit' "$replacement"
