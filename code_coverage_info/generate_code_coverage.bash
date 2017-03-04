#!/bin/bash

cd `git rev-parse --show-toplevel`
echo $PWD
echo "Generating Docs"
doxygen > /dev/null # Generates docs
echo "Coverage report"
python -m coverxygen --xml-dir ./docs/xml --src-dir . --output ./code-coverage-info/doc-coverage.info > /dev/null
echo "Generate HTML Report"
genhtml --no-function-coverage --no-branch-coverage code-coverage-info/doc-coverage.info -o ./code-coverage-info > /dev/null
echo "Print Summary"
lcov --summary ./code-coverage-info/doc-coverage.info
