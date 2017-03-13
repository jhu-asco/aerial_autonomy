#!/bin/bash
# Script to convert a given file to clang format
#

# Get base name without path
function clang_format_file {
  arg=$1
  file_name=${arg##*/}
  echo "Copying $file_name to temp file"
  cp $1 /tmp/$file_name
  clang-format /tmp/$file_name > $1
}

if [[ -f $1 ]];
then
  echo "Formatting file $1"
  clang_format_file $1 
elif [[ -d $1 ]];
then
  echo "Formatting files in directory $1"
  file_list=$(find $1 \( -name '*.cpp' -o -name '*.h' -o -name '*.C' -o -name '*.H', -o -name '*.hpp'  \))
  for file_name in $file_list; do
    echo "Formatting file $file_name"
    clang_format_file $file_name
  done
else
  echo "Usage: clang_format_file.bash [NAME]
        NAME: File or folder name
          If NAME is a file, the file is clang formatted
          else if NAME is a directory, all the c++ files
          matching *.h, *.hpp, *.H *.C, *.cpp will be
          formatted into clang format 
       "
fi
