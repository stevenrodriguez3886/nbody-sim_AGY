#########################################
# any aliases we need can be stored here
#########################################
alias gs='git status'
alias cmCP='cmake -S . -B build ; cmake --build build -j"$(nproc)"'  # compile sfml program with cmake. Stands for "cmake Compile Program"
alias ms='rm -rf build build-coverage coverage-html coverage.info'