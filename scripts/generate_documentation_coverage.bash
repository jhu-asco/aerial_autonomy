#!/bin/bash

cd `git rev-parse --show-toplevel`
echo $PWD
echo "Generating Docs"
doxygen > /dev/null # Generates docs
# Create coverage folder if it does not exist
[ -d documentation_coverage_info ] || mkdir documentation_coverage_info
echo "Coverage report"
python -m coverxygen --xml-dir ./docs/xml --src-dir . --output ./documentation_coverage_info/doc_coverage.info > /dev/null
echo "Generate HTML Report"
genhtml --no-function-coverage --no-branch-coverage documentation_coverage_info/doc_coverage.info -o ./documentation_coverage_info > /dev/null
echo "Print Summary"
lcov --summary ./documentation_coverage_info/doc_coverage.info
