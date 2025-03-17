# Function to update terminal background color based on IP segment
function set_bg_by_ip() {
    # Get the first IP starting with 10.42.
    local ip
    ip=$(hostname -I | tr ' ' '\n' | grep "^10\.42\." | head -n 1)

    # Default color: black
    local color="#000000"
    
    if [ -n "$ip" ]; then
        # Extract the third octet
        IFS='.' read -r _ _ oct3 _ <<< "$ip"
        if [ "$oct3" -ne 0 ]; then
            color="#003020"  # Light green
        fi
    fi

    # Set the terminal background using OSC 11 (xterm-compatible)
    printf "\033]11;%s\007" "$color"
}

# Run this function every time before displaying the prompt
PROMPT_COMMAND="set_bg_by_ip; $PROMPT_COMMAND"
