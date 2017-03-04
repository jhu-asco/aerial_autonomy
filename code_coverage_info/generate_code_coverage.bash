#!/bin/bash

cd `git rev-parse --show-toplevel`
echo $PWD
echo "Generating Docs"
doxygen > /dev/null # Generates docs
echo "Coverage report"
python -m coverxygen --xml-dir ./docs/xml --src-dir . --output ./code_coverage_info/doc_coverage.info > /dev/null
echo "Generate HTML Report"
genhtml --no-function-coverage --no-branch-coverage code_coverage_info/doc_coverage.info -o ./code_coverage_info > /dev/null
echo "Print Summary"
lcov --summary ./code_coverage_info/doc_coverage.info
