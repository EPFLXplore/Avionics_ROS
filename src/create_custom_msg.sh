
# @file create_custom_msg.sh
# @author Eliot Abramo

# Set the source file path.
SOURCE="avionics_costco/SerialHandler/Packets/generate_structs.cpp"

# Set the output executable name.
EXE="generate_structs"

COMPILE_FLAGS="-std=c++17 -DGENERATE_MSG"

# Compile the source file with C++17 and the macro definition.
echo "Compiling $SOURCE with GENERATE_MSG..."
g++ $COMPILE_FLAGS -o "$EXE" "$SOURCE"
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

# Define input folder and output file for the executable.
INPUT_FOLDER="msg/avionics/"
OUTPUT_FILE="all_structs.generated.hpp"

# Execute the compiled program.
echo "Running $EXE with input folder '$INPUT_FOLDER' and output file '$OUTPUT_FILE'..."
./"$EXE" "$INPUT_FOLDER" "$OUTPUT_FILE"
