#!/bin/zsh
# 1. Load your full interactive profile (gets all env vars)
# The -i flag forces interactive mode to load .zshrc
source ~/.zshrc

# 2. Print env to verify (optional, appears in Xcode console)
echo "Launching with PATH: $PATH"

# 3. Launch the binary, passing any arguments forwarded by Xcode ($@)
# "exec" replaces the shell process with your app process (cleaner for profiling)
exec ./build/fsai_run "$@"
