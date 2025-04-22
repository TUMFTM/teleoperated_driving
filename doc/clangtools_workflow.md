# Optional Static Analysis Via Clang Tools

This guide covers the setup and usage of ClangFormat and ClangTidy for code formatting and static analysis in C++ projects.


## About ClangFormat and ClangTidy

### ClangFormat
- Automated code formatting tool
- Configurable via .clang-format file
- Ensures consistent code style across your project
- Can be integrated into CI/CD pipelines

### ClangTidy

- Static code analysis tool
- Finds programming errors, style violations, and bugs
- Configurable via .clang-tidy file
- Requires compilation database (compile_commands.json)

## Preparation



## Install / Dependencies 

- Install LLVM and Clang Tools

'''bash
# Add LLVM repository key (modern method)
wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | sudo tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc

# Add LLVM repository
sudo add-apt-repository "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-18 main"

# Update package list
sudo apt update

# Install required packages
sudo apt-get install clang-18 clang-tools-18 clang-18-doc libclang-common-18-dev libclang-18-dev libclang1-18 clang-format-18 python3-clang-18 clangd-18 clang-tidy-18
'''

## Linting via Clang-Format

use script run_clang_format.sh
1. Use Clang Format from the tod foler and give it a Dir or FilePath
'''bash
./run_clang_format.sh <path_to_dir_or_file>
'''


## Static Analysis via ClangTidy

1. Build The Projects with CompileCommands on 
'''bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
'''
2. Use Clang Tidy from the tod foler and give it a Dir or FilePath
'''bash
./run_clang_tidy.sh <path_to_dir_or_file>
'''
